import itertools

import signal
import csv
from datetime import datetime
import os

import pygame
import numpy as np
from swarmsim.agent.control.AbstractController import AbstractController
from swarmsim.agent.MazeAgent import MazeAgent

import scipy as sp
from scipy import spatial
import sys
from swarmsim.util.pid import PID

# typing
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...world.RectangularWorld import RectangularWorld
else:
    RectangularWorld = None

# Try to import numba for JIT compilation
try:
    from numba import jit
    NUMBA_AVAILABLE = True
except ImportError:
    NUMBA_AVAILABLE = False
    # Fallback: no-op decorator
    def jit(*args, **kwargs):
        def decorator(func):
            return func
        return decorator

"""
VoronoiController - PAPER FAITHFUL IMPLEMENTATION

Implements paper: "Self-triggered coordination of robotic networks for optimal deployment"
by Cameron Nowzari and Jorge Cortés (Automatica 48 (2012) 1077–1087)

Paper algorithms implemented:
    DONE:
    Algorithm 1: tbb() motion control (to-ball-boundary map)
    Algorithm 2: One-step-ahead update policy with proper condition
    Table 5: Self-triggered centroid algorithm (complete)
    Table 4: Voronoi cell computation (expanding radius neighbor discovery)
    Dynamic uncertainty: r_ij = v_max * τ_ij
    Hyperbolic boundaries (Equations 6 & 9)
    Per-agent active set A^i (only tracks neighbors, not all agents)
    
    not done:
    Approximate bnd() bound (needs polygonal cells for exact version)
    Grid-sampled guaranteed centroids (needs polygons for exact version)

Key changes for paper faithfulness:
    - Removed goal_cache (paper recomputes q fresh every timestep)
    - D^i_i now stores future position (where tbb says to go), not current
    - Always compute fresh q and r with current D before moving
    - Performance optimizations kept where they don't contradict paper
"""

# JIT-compiled helper functions
@jit(nopython=True)
def _test_guaranteed_cell_numba(points, agent_pos, r_i, other_pos_array, other_r_array):
    """Numba-optimized guaranteed cell membership test"""
    n_points = points.shape[0]
    n_agents = other_pos_array.shape[0]
    is_guaranteed = np.ones(n_points, dtype=np.bool_)
    
    for i in range(n_points):
        px, py = points[i, 0], points[i, 1]
        max_dist_to_i = np.sqrt((px - agent_pos[0])**2 + (py - agent_pos[1])**2) + r_i
        
        for j in range(n_agents):
            ox, oy = other_pos_array[j, 0], other_pos_array[j, 1]
            dist_to_j = np.sqrt((px - ox)**2 + (py - oy)**2)
            min_dist_to_j = max(0.0, dist_to_j - other_r_array[j])
            
            if max_dist_to_i > min_dist_to_j:
                is_guaranteed[i] = False
                break
    
    return is_guaranteed


class VoronoiController(AbstractController):
    # Class-level shared H history (all agents contribute to same global H)
    _shared_H_history = []
    _last_H_record_time = -1.0
    _H_record_interval = 0.1  # Record H every 0.1 seconds
    _output_dir = "voronoi_data"  # Directory to save data

    def __init__(self, agent=None, parent=None):
        self.world = None
        self.population = None
        self.world_size = None
        self.world_radius = None
        self.world_dt = None
        self.allpairs = []
        self.lines = []
        self.centroids = []
        
        # PAPER FAITHFUL: Per-agent data structures
        # agent_memory now only contains agents in A^i (active set)
        self.agent_memory = {}  # {agent_id: {'last_known_pos', 'last_update_time', 'uncertainty_radius'}}
        self.active_set = set()  # A^i: set of agent IDs whose information we maintain
        self.current_time = 0
        
        # Visualization control
        self.show_mode = "guaranteed"  # Options: "guaranteed", "dual_guaranteed", "all"
        
        # Fixed uncertainty radius (when not using dynamic)
        self.fixed_uncertainty_radius = 0.3
        
        # Dynamic uncertainty parameters
        self.use_dynamic_uncertainty = True
        self.epsilon = 0.55 # was 0.1, 0.55 is the paper's value, works better
        self.v_max = 0.15
        self.k_neighbors = 4
        
        # Visualization flags
        self.draw_voronoi = True
        self.draw_uncertainty_circles = True
        
        # Store computed boundaries (for visualization - global view)
        # Note: These are for visualization only, not used by individual agent
        self.guaranteed_boundaries = []
        self.dual_guaranteed_boundaries = []
        self.guaranteed_centroids = {}
        
        # Shared Voronoi cache (for visualization)
        self.voronoi_cache_time = -1
        self.cached_vor = None
        self.cached_centroids = {}
        
        # OPTIMIZATION: Centroid caching with uncertainty fingerprints
        # This is OK because it only caches when uncertainty state is identical
        self.centroid_cache = {}  # {agent_id: (fingerprint, centroid, timestamp)}
        self.last_valid_centroid = {}  # {agent_id: last_successful_centroid} - for fallback stability
        
        # REMOVED: goal_cache - paper requires computing q fresh every timestep
        # REMOVED: centroid_update_interval - paper computes fresh every time
        # REMOVED: should_update_centroid() - paper always computes fresh
        
        # OPTIMIZATION: Stationary detection (doesn't contradict paper)
        self.last_positions = {}  # {agent_id: last_position}
        self.movement_threshold = 0.01  # Consider stationary if movement < this
        self.system_is_stable = False
        
        # Paper Algorithm parameters
        self.use_tbb_motion = True  # Use paper's tbb() motion control
        self.use_paper_update_policy = True  # Use Algorithm 2's proper condition
        self.use_expanding_radius = True  # Use Table 4 for neighbors (paper's method)
        
        # Motion control parameters
        self.deceleration_radius = 1.0  # Start slowing down when within this distance
        self.goal_tolerance = 0.1  # Stop when within this distance of goal

        # Track H over time
        self.density_function = None
        
        # Create output directory if it doesn't exist
        if not os.path.exists(VoronoiController._output_dir):
            os.makedirs(VoronoiController._output_dir)
            print(f"Created output directory: {VoronoiController._output_dir}")
        
        # Register signal handler for graceful shutdown (Ctrl+C)
        if not hasattr(VoronoiController, '_signal_registered'):
            signal.signal(signal.SIGINT, VoronoiController._signal_handler)
            VoronoiController._signal_registered = True
            print("Press Ctrl+C to stop simulation and save data")
        
        self.tracking_pid = PID(p=0.3, i=0.005, d=0.1)
        super().__init__(agent=agent, parent=parent)
    
    @classmethod
    def _signal_handler(cls, sig, frame):
        """Handle Ctrl+C gracefully - save data before exiting"""
        print("\n\n" + "="*60)
        print("Simulation interrupted - saving data...")
        print("="*60)
        
        # Save H history to CSV
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = cls.save_H_history_csv(timestamp=timestamp)
        
        # Print statistics
        stats = cls.get_H_statistics()
        if stats:
            print(f"\nH Statistics:")
            print(f"  Initial H: {stats['initial_H']:.4f}")
            print(f"  Final H: {stats['final_H']:.4f}")
            print(f"  Reduction: {stats['reduction']:.4f} ({stats['percent_reduction']:.1f}%)")
            print(f"  Simulation time: {stats['convergence_time']:.1f}s")
            print(f"  Total samples: {stats['num_samples']}")
        
        print(f"\nData saved to: {filename}")
        print("="*60)
        sys.exit(0)
    
    def set_density_function(self, density_fn):
        """Set the density function φ(q) for computing H"""
        self.density_function = density_fn
    
    def compute_H(self):
        """
        Compute aggregate distortion H(P, V(P)) from paper's equation (2):
        H(P, V(P)) = Σᵢ ∫_{Vᵢ} ||q - pᵢ||² φ(q) dq
        """
        if self.world_size is None or self.population is None:
            return 0.0
        
        # Use uniform density if none provided
        if self.density_function is None:
            def uniform_density(x, y):
                return 1.0
            density_fn = uniform_density
        else:
            density_fn = self.density_function
        
        # Compute Voronoi partition
        vor, centroids_dict = self.compute_voronoi_once_per_frame()
        
        if vor == []:
            return 0.0
        
        H_total = 0.0
        
        # For each agent's Voronoi cell
        for i, region in enumerate(vor.filtered_regions):
            vertices = vor.vertices[region]
            
            # Find which agent this cell belongs to
            region_center = np.mean(vertices, axis=0)
            distances = [np.linalg.norm(region_center - point) for point in vor.filtered_points]
            closest_point_idx = np.argmin(distances)
            agent_pos = vor.filtered_points[closest_point_idx]
            
            # Monte Carlo integration
            min_x, max_x = np.min(vertices[:, 0]), np.max(vertices[:, 0])
            min_y, max_y = np.min(vertices[:, 1]), np.max(vertices[:, 1])
            
            n_samples = 2000 # was 500 samples per H computation, more samples = more accurate H but slower
            samples_x = np.random.uniform(min_x, max_x, n_samples)
            samples_y = np.random.uniform(min_y, max_y, n_samples)
            
            # Check which samples are inside the polygon
            from matplotlib.path import Path
            cell_polygon = Path(vertices)
            sample_points = np.column_stack([samples_x, samples_y])
            inside = cell_polygon.contains_points(sample_points)
            
            H_cell = 0.0
            n_inside = np.sum(inside)
            
            if n_inside > 0:
                for idx in np.where(inside)[0]:
                    x, y = samples_x[idx], samples_y[idx]
                    dist_squared = (x - agent_pos[0])**2 + (y - agent_pos[1])**2
                    phi_value = density_fn(x, y)
                    H_cell += dist_squared * phi_value
                
                bbox_area = (max_x - min_x) * (max_y - min_y)
                H_cell = (bbox_area / n_inside) * H_cell
            
            H_total += H_cell
        
        return H_total
    
    def maybe_record_H(self):
        """Record H periodically (not every frame for performance)"""
        if self.current_time - VoronoiController._last_H_record_time < VoronoiController._H_record_interval:
            return
        
        VoronoiController._last_H_record_time = self.current_time
        
        try:
            H_value = self.compute_H()
            VoronoiController._shared_H_history.append((self.current_time, H_value))
            
            # Print progress occasionally
            if len(VoronoiController._shared_H_history) % 50 == 0:
                print(f"[t={self.current_time:.1f}s] Recorded {len(VoronoiController._shared_H_history)} samples, H={H_value:.4f}")
        except Exception as e:
            print(f"Warning: Failed to compute H: {e}")
    
    @classmethod
    def save_H_history_csv(cls, timestamp=None):
        """
        Save H history to CSV file
        
        Returns:
            str: Filename of saved CSV
        """
        if not cls._shared_H_history:
            print("No H history to save.")
            return None
        
        if timestamp is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        filename = os.path.join(cls._output_dir, f"H_history_{timestamp}.csv")
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'H'])  # Header
            writer.writerows(cls._shared_H_history)
        
        return filename
    
    @classmethod
    def save_checkpoint(cls, prefix="checkpoint"):
        """Save current H history without stopping simulation"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(cls._output_dir, f"{prefix}_{timestamp}.csv")
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'H'])
            writer.writerows(cls._shared_H_history)
        
        print(f"Checkpoint saved: {filename}")
        return filename
    
    @classmethod
    def get_H_statistics(cls):
        """Get statistics about H convergence"""
        if not cls._shared_H_history:
            return None
        
        times, H_values = zip(*cls._shared_H_history)
        
        return {
            'initial_H': H_values[0],
            'final_H': H_values[-1],
            'reduction': H_values[0] - H_values[-1],
            'percent_reduction': 100 * (H_values[0] - H_values[-1]) / H_values[0] if H_values[0] > 0 else 0,
            'convergence_time': times[-1],
            'num_samples': len(H_values)
        }
    
    @classmethod
    def reset_H_history(cls):
        """Clear H history (for starting new runs)"""
        cls._shared_H_history = []
        cls._last_H_record_time = -1.0
    
    @classmethod
    def set_H_record_interval(cls, interval):
        """Set how often to record H (in seconds)"""
        cls._H_record_interval = interval
    
    @classmethod
    def set_output_directory(cls, directory):
        """Set where to save output files"""
        cls._output_dir = directory
        if not os.path.exists(directory):
            os.makedirs(directory)

    def attach_world(self, world: RectangularWorld):
        self.world = world
        self.population = world.population
        self.world_size = world.config.size
        self.world_radius = world.config.radius
        self.world_dt = world.dt
        
        if self.agent:
            agent_id = int(self.agent.name)
            agent_pos = np.array(self.agent.getPosition())
            
            # PAPER FAITHFUL: At t=0, agent has perfect information about ALL agents
            # This is the initial "global communication" before the algorithm starts
            print(f"Agent {agent_id}: Initial global communication (t=0)")
            for pop_agent in self.population:
                other_id = int(pop_agent.name)
                self.agent_memory[other_id] = {
                    'last_known_pos': np.array(pop_agent.getPosition()),
                    'last_update_time': 0.0,
                    'uncertainty_radius': 0.0  # Perfect info at t=0
                }
            
            # Temporarily, active_set contains all agents (for Voronoi cell computation)
            self.active_set = set(int(a.name) for a in self.population)
            
            # PAPER'S TABLE 5, Initialization: Execute Voronoi cell computation
            # This determines which neighbors are actually relevant (A^i)
            print(f"Agent {agent_id}: Running initial Voronoi cell computation...")
            relevant_neighbors = self.voronoi_cell_computation_paper(self.agent)
            
            # Now keep ONLY relevant neighbors in active set and memory
            self.active_set = relevant_neighbors
            
            # Remove agents not in active set from memory
            agents_to_remove = [aid for aid in self.agent_memory.keys() 
                            if aid not in self.active_set]
            for aid in agents_to_remove:
                del self.agent_memory[aid]
            
            print(f"Agent {agent_id}: Initialized with |A^i|={len(self.active_set)} "
                f"(kept {len(self.active_set)}/{len(self.population)} agents)")
        
        # Print configuration
        print("=" * 60)
        print("VoronoiController Configuration:")
        print(f"  Paper algorithms:")
        print(f"    - Algorithm 1 (tbb motion): {self.use_tbb_motion}")
        print(f"    - Algorithm 2 (update policy): {self.use_paper_update_policy}")
        print(f"    - Table 4 (Voronoi cell computation): {self.use_expanding_radius}")
        print(f"    - Table 5 (self-triggered centroid): enabled")
        print(f"  Dynamic uncertainty: {self.use_dynamic_uncertainty}")
        print(f"  Numba JIT: {'available' if NUMBA_AVAILABLE else 'not available (using numpy)'}")
        print(f"  Parameters: ε={self.epsilon}, v_max={self.v_max}")
        print(f"  Per-agent active set A^i: enabled")
        print(f"  Optimizations: centroid caching (uncertainty fingerprint)")
        print("=" * 60)

    def update_agent_memory(self, current_agent):
        """
        Update memory with current agent position and uncertainty radii
        PAPER FAITHFUL: Only updates agents in active set A^i, not all agents
        
        Note: D^i_i will be updated with FUTURE position after tbb() in get_actions()
        This function just updates uncertainty radii for neighbors
        """
        if self.world_size is None:
            return
        
        # Update time
        if hasattr(self.world, 'time'):
            self.current_time = self.world.time
        else:
            self.current_time += self.world_dt if self.world_dt else 0.025
        
        current_agent_id = int(current_agent.name)
        current_pos = np.array(current_agent.getPosition())
        
        # OPTIMIZATION: Track movement for stationary detection
        if current_agent_id in self.last_positions:
            movement = np.linalg.norm(current_pos - self.last_positions[current_agent_id])
            if movement < self.movement_threshold:
                self.system_is_stable = True
            else:
                self.system_is_stable = False
        self.last_positions[current_agent_id] = current_pos.copy()
        
        # PAPER FAITHFUL: Update uncertainty ONLY for agents in active set A^i
        # Note: We don't update D^i_i here - that's done after tbb() computes next position
        max_uncertainty = 0.0
        agents_to_remove = []
        
        for agent_id in self.active_set:
            if agent_id == current_agent_id:
                continue  # Don't update self here - done after tbb()
            
            if agent_id not in self.agent_memory:
                # This shouldn't happen, but handle gracefully
                agents_to_remove.append(agent_id)
                continue
            
            time_elapsed = self.current_time - self.agent_memory[agent_id]['last_update_time']
            uncertainty_radius = self.v_max * time_elapsed
            
            max_world_uncertainty = np.sqrt(self.world_size[0]**2 + self.world_size[1]**2)
            uncertainty_radius = min(uncertainty_radius, max_world_uncertainty)
            
            self.agent_memory[agent_id]['uncertainty_radius'] = uncertainty_radius
            max_uncertainty = max(max_uncertainty, uncertainty_radius)
        
        # Clean up agents that shouldn't be in active set
        for agent_id in agents_to_remove:
            self.active_set.discard(agent_id)
        
        # Debug output (occasionally)
        if int(self.current_time * 10) % 50 == 0:
            print(f"Agent {current_agent_id} at t={self.current_time:.2f}: "
                  f"|A^i|={len(self.active_set)}, max_unc={max_uncertainty:.3f}, "
                  f"stable={self.system_is_stable}")

    def tbb(self, p, delta, q, r):
        """
        Algorithm 1: To-ball-boundary map
        Move from p toward q by distance delta, but stop at boundary of B(q, r)
        
        Args:
            p: current position (numpy array)
            delta: maximum movement distance (vmax * dt)
            q: target position (centroid)
            r: bound radius
        
        Returns:
            new position (numpy array)
        """
        p = np.array(p)
        q = np.array(q)
        
        dist_to_q = np.linalg.norm(p - q)
        
        if dist_to_q <= 1e-6:  # Already at target
            return p
        
        # Distance from p to boundary of ball B(q, r)
        dist_to_boundary = max(0, dist_to_q - r)
        
        if dist_to_boundary >= delta:
            # Far from ball: move toward q by distance delta
            direction = (q - p) / dist_to_q
            return p + delta * direction
        else:
            # Close to ball: project onto boundary B(q, r)
            if dist_to_q > r:
                # Outside ball, move to boundary
                direction = (q - p) / dist_to_q
                return q + r * direction
            else:
                # Inside ball, stay put (shouldn't happen normally)
                return p

    def compute_bnd_approximate(self, agent_id):
        """
        Approximate bound computation for Algorithm 2
        
        This is a simplified version since we don't have exact polygonal cells yet.
        Uses the guaranteed centroid distance as a proxy.
        
        Full implementation needs: bnd = 2*cr(dgVi) * (1 - M_gVi/M_dgVi)
        """
        if agent_id not in self.agent_memory:
            return float('inf')
        
        # Get maximum uncertainty in the active set (not all agents!)
        max_uncertainty = max(
            (info['uncertainty_radius'] for aid, info in self.agent_memory.items() 
             if aid in self.active_set and aid != agent_id),
            default=0.0
        )
        
        # Approximate bound: related to uncertainty and neighbor distances
        if agent_id in self.agent_memory:
            agent_pos = self.agent_memory[agent_id]['last_known_pos']
            
            # Find closest neighbor distance (from active set only)
            min_neighbor_dist = float('inf')
            for other_id in self.active_set:
                if other_id == agent_id:
                    continue
                if other_id not in self.agent_memory:
                    continue
                other_pos = np.array(self.agent_memory[other_id]['last_known_pos'])
                dist = np.linalg.norm(agent_pos - other_pos)
                min_neighbor_dist = min(min_neighbor_dist, dist)
            
            # Heuristic: bound is proportional to uncertainty and inversely to neighbor distance
            # This approximates the paper's circumradius-based bound
            if min_neighbor_dist < float('inf'):
                bnd = 2 * max_uncertainty * (1 + max_uncertainty / min_neighbor_dist)
            else:
                bnd = 2 * max_uncertainty
            
            return bnd
        
        return float('inf')

    def should_update_information(self, current_agent):
        """
        Algorithm 2: One-step-ahead update policy
        
        Paper's condition (line 4): if r >= max{||q - pi||, ε} then update
        where r = bnd(gVi, dgVi) and q = CgVi
        
        PAPER FAITHFUL: Always computes fresh q with current D
        """
        if not self.use_paper_update_policy:
            # Original simple version
            current_agent_id = int(current_agent.name)
            for agent_id in self.active_set:
                if agent_id == current_agent_id:
                    continue
                if agent_id not in self.agent_memory:
                    continue
                if self.agent_memory[agent_id]['uncertainty_radius'] > self.epsilon:
                    return True
            return False
        
        # Paper's Algorithm 2
        current_agent_id = int(current_agent.name)
        agent_pos = np.array(current_agent.getPosition())
        
        # PAPER FAITHFUL: Always compute fresh q based on current uncertainty
        q = self.compute_guaranteed_voronoi_centroid_fast(current_agent_id, agent_pos)
        
        # Compute bound r = bnd(gVi, dgVi)
        r = self.compute_bnd_approximate(current_agent_id)
        
        # Distance to guaranteed centroid
        dist_to_centroid = np.linalg.norm(q - agent_pos)
        
        # Algorithm 2, line 4: if r >= max{||q - pi||, ε}
        return r >= max(dist_to_centroid, self.epsilon)

    def voronoi_cell_computation_paper(self, current_agent):
        """
        Paper's Table 4: Voronoi cell computation
        This determines which agents should be in the active set A^i
        by gradually expanding the search radius until the Voronoi cell is complete.
        
        Returns:
            set of agent IDs that should be in A^i (including self)
        """
        current_agent_id = int(current_agent.name)
        agent_pos = np.array(current_agent.getPosition())
        
        # Line 1: Initialize R_i
        # R_i = min over neighbors of (||pi - pj|| + vmax*τ_j)
        min_dist = float('inf')
        for other_id in self.active_set:
            if other_id == current_agent_id:
                continue
            if other_id in self.agent_memory:
                other_pos = self.agent_memory[other_id]['last_known_pos']
                uncertainty = self.agent_memory[other_id]['uncertainty_radius']
                dist = np.linalg.norm(agent_pos - other_pos) + uncertainty
                min_dist = min(min_dist, dist)
        
        # If we have no neighbors in active set, start with a default radius
        if min_dist == float('inf'):
            Ri = min(2.0, self.world_size[0] / 2)  # Start with reasonable radius
        else:
            Ri = min_dist
        
        # Lines 2-3: Detect neighbors and compute initial cell
        neighbors_in_radius = set()
        
        # Lines 4-8: Expand radius until condition met
        max_iterations = 10  # Prevent infinite loops
        iteration = 0
        
        while iteration < max_iterations:
            # Detect all agents within radius R_i
            neighbors_in_radius = {current_agent_id}  # Include self
            
            for pop_agent in self.population:
                other_id = int(pop_agent.name)
                if other_id == current_agent_id:
                    continue
                
                other_pos = np.array(pop_agent.getPosition())
                dist = np.linalg.norm(agent_pos - other_pos)
                
                if dist <= Ri:
                    neighbors_in_radius.add(other_id)
            
            # Check stopping condition
            # Find closest neighbor to determine cell extent
            closest_neighbor_dist = float('inf')
            for other_id in neighbors_in_radius:
                if other_id == current_agent_id:
                    continue
                
                # Find this agent in population
                for pop_agent in self.population:
                    if int(pop_agent.name) == other_id:
                        other_pos = np.array(pop_agent.getPosition())
                        dist = np.linalg.norm(agent_pos - other_pos)
                        closest_neighbor_dist = min(closest_neighbor_dist, dist)
                        break
            
            # Approximate cell extent as midpoint to closest neighbor
            if closest_neighbor_dist < float('inf'):
                max_cell_extent = closest_neighbor_dist / 2
            else:
                max_cell_extent = Ri
            
            # Condition: R_i >= 2 * max distance from pi to cell boundary
            if Ri >= 2 * max_cell_extent:
                break
            
            # Expand radius
            Ri = 2 * Ri
            iteration += 1
        
        # Lines 9-10: Return active set A_i = N_i ∪ {i}
        return neighbors_in_radius

    def trigger_information_update(self, current_agent):
        """
        PAPER FAITHFUL: Paper's Table 5, step 5
        When update is needed, run Voronoi cell computation to find neighbors
        and get their current positions (simulates communication)
        """
        if self.should_update_information(current_agent):
            current_agent_id = int(current_agent.name)
            
            # Paper's Table 5, step 5: "reset D^i by running Voronoi cell computation"
            # This gives us the new active set A^i
            new_active_set = self.voronoi_cell_computation_paper(current_agent)
            
            if self.use_paper_update_policy:
                # Show which condition triggered it
                agent_pos = np.array(current_agent.getPosition())
                q = self.compute_guaranteed_voronoi_centroid_fast(current_agent_id, agent_pos)
                r = self.compute_bnd_approximate(current_agent_id)
                dist = np.linalg.norm(q - agent_pos)
                print(f"Agent {current_agent_id}: Update triggered (Algorithm 2) - "
                      f"bnd={r:.3f} >= max(dist={dist:.3f}, ε={self.epsilon})")
            else:
                print(f"Agent {current_agent_id}: Update triggered at t={self.current_time:.2f}, "
                      f"|A^i|: {len(self.active_set)} -> {len(new_active_set)}")
            
            # Update active set
            old_active_set = self.active_set.copy()
            self.active_set = new_active_set
            
            # Get current positions for all agents in new active set
            # This simulates "communication" with neighbors
            for agent_id in self.active_set:
                if agent_id == current_agent_id:
                    continue  # Don't update self here - done after tbb()
                
                # Find this agent in population and get its current position
                for pop_agent in self.population:
                    if int(pop_agent.name) == agent_id:
                        self.agent_memory[agent_id] = {
                            'last_known_pos': np.array(pop_agent.getPosition()),
                            'last_update_time': self.current_time,
                            'uncertainty_radius': 0.0
                        }
                        break
            
            # Remove agents no longer in active set from memory
            agents_to_remove = old_active_set - self.active_set
            for agent_id in agents_to_remove:
                if agent_id in self.agent_memory:
                    del self.agent_memory[agent_id]
            
            # Clear caches after update (centroid cache is OK, uses fingerprint)
            self.centroid_cache.clear()
            self.system_is_stable = False

    def get_k_nearest_neighbors(self, agent_id, k=None):
        """
        Get k nearest neighbors from ACTIVE SET only
        
        If use_expanding_radius=True, uses Table 4 (Voronoi cell computation)
        Otherwise uses K-nearest from active set (faster approximation)
        """
        if self.use_expanding_radius:
            # Paper's Table 4 already determined neighbors in active set
            # Just return active set minus self
            return [aid for aid in self.active_set if aid != agent_id]
        
        # K-nearest approach - but only from active set
        if k is None:
            k = self.k_neighbors
        
        if agent_id not in self.agent_memory:
            return []
        
        agent_pos = np.array(self.agent_memory[agent_id]['last_known_pos'])
        
        distances = []
        for other_id in self.active_set:
            if other_id == agent_id:
                continue
            
            if other_id not in self.agent_memory:
                continue
            
            other_pos = np.array(self.agent_memory[other_id]['last_known_pos'])
            dist = np.linalg.norm(agent_pos - other_pos)
            distances.append((dist, other_id))
        
        distances.sort()
        return [other_id for _, other_id in distances[:min(k, len(distances))]]

    def compute_uncertainty_fingerprint(self, agent_id):
        """Create hashable fingerprint of uncertainty state (from active set only)"""
        # Round to 2 decimal places to avoid cache misses from tiny changes
        return tuple(sorted(
            (aid, round(info['uncertainty_radius'], 2))
            for aid, info in self.agent_memory.items()
            if aid in self.active_set
        ))

    def compute_guaranteed_voronoi_centroid_fast(self, agent_id, agent_pos):
        """
        OPTIMIZED: Vectorized + cached + culled computation
        PAPER FAITHFUL: Only uses agents in active set A^i
        
        Caching is OK because it uses uncertainty fingerprint - if D unchanged, result unchanged
        """
        if self.world_size is None:
            return np.array(agent_pos)
        
        agent_pos = np.array(agent_pos)
        
        if agent_id not in self.agent_memory or agent_id not in self.active_set:
            return agent_pos
        
        # OPTIMIZATION: Check cache first (OK because fingerprint ensures D hasn't changed)
        current_fingerprint = self.compute_uncertainty_fingerprint(agent_id)
        if agent_id in self.centroid_cache:
            cached_fingerprint, cached_centroid, _ = self.centroid_cache[agent_id]
            if cached_fingerprint == current_fingerprint:
                return cached_centroid
        
        # OPTIMIZATION 2: Check if uncertainty is significant (in active set)
        max_uncertainty = max(
            (info['uncertainty_radius'] for aid, info in self.agent_memory.items() 
             if aid in self.active_set and aid != agent_id),
            default=0.0
        )
        
        # If no uncertainty, use regular Voronoi centroid instead
        if max_uncertainty < 0.001:  # Very small threshold
            # Get regular Voronoi centroid
            vor, centroids_dict = self.compute_voronoi_once_per_frame()
            if agent_id in centroids_dict:
                centroid = centroids_dict[agent_id]
                self.centroid_cache[agent_id] = (current_fingerprint, centroid, self.current_time)
                self.last_valid_centroid[agent_id] = centroid  # Store as valid
                return centroid
            else:
                # Fallback to last valid centroid, or agent_pos if never computed
                return self.last_valid_centroid.get(agent_id, agent_pos)
        
        # OPTIMIZATION 3: Adaptive resolution
        if max_uncertainty < 0.05:
            resolution = 0.3
            search_radius = 1.0
        elif max_uncertainty < 0.15:
            resolution = 0.25
            search_radius = 1.5
        else:
            resolution = 0.2
            search_radius = 2.0
        
        # OPTIMIZATION 4: Spatial culling - only consider relevant neighbors FROM ACTIVE SET
        relevant_neighbors = [
            (aid, np.array(info['last_known_pos']), info['uncertainty_radius'])
            for aid, info in self.agent_memory.items()
            if aid in self.active_set and  # PAPER FAITHFUL: only active set
               aid != agent_id and 
               np.linalg.norm(np.array(info['last_known_pos']) - agent_pos) < search_radius * 2
        ]
        
        if not relevant_neighbors:
            # No neighbors, use regular Voronoi
            vor, centroids_dict = self.compute_voronoi_once_per_frame()
            if agent_id in centroids_dict:
                centroid = centroids_dict[agent_id]
                self.centroid_cache[agent_id] = (current_fingerprint, centroid, self.current_time)
                self.last_valid_centroid[agent_id] = centroid
                return centroid
            else:
                return self.last_valid_centroid.get(agent_id, agent_pos)
        
        # Create sampling grid
        x_min = max(0, agent_pos[0] - search_radius)
        x_max = min(self.world_size[0], agent_pos[0] + search_radius)
        y_min = max(0, agent_pos[1] - search_radius)
        y_max = min(self.world_size[1], agent_pos[1] + search_radius)
        
        x_range = np.arange(x_min, x_max, resolution)
        y_range = np.arange(y_min, y_max, resolution)
        
        # OPTIMIZATION 5: Vectorized meshgrid (no nested loops in Python)
        X, Y = np.meshgrid(x_range, y_range)
        points = np.stack([X.ravel(), Y.ravel()], axis=1)
        
        # Get agent's uncertainty
        r_i = self.agent_memory[agent_id]['uncertainty_radius']
        
        # OPTIMIZATION 6: Use Numba if available, otherwise vectorized numpy
        if NUMBA_AVAILABLE and len(relevant_neighbors) > 0:
            # Prepare data for Numba
            other_pos_array = np.array([pos for _, pos, _ in relevant_neighbors])
            other_r_array = np.array([r for _, _, r in relevant_neighbors])
            
            is_guaranteed = _test_guaranteed_cell_numba(
                points, agent_pos, r_i, other_pos_array, other_r_array
            )
        else:
            # Vectorized numpy fallback
            dists_to_i = np.linalg.norm(points - agent_pos, axis=1) + r_i
            
            is_guaranteed = np.ones(len(points), dtype=bool)
            for _, other_pos, r_j in relevant_neighbors:
                dists_to_j = np.linalg.norm(points - other_pos, axis=1) - r_j
                dists_to_j = np.maximum(dists_to_j, 0)  # Clamp to 0
                is_guaranteed &= (dists_to_i <= dists_to_j)
        
        guaranteed_points = points[is_guaranteed]
        
        if len(guaranteed_points) > 0:
            centroid = np.mean(guaranteed_points, axis=0)
            self.last_valid_centroid[agent_id] = centroid  # Store successful computation
        else:
            # Fallback: Try regular Voronoi first, then last valid, then agent position
            vor, centroids_dict = self.compute_voronoi_once_per_frame()
            if agent_id in centroids_dict:
                centroid = centroids_dict[agent_id]
                self.last_valid_centroid[agent_id] = centroid
            elif agent_id in self.last_valid_centroid:
                # Use last successful computation - prevents oscillation
                centroid = self.last_valid_centroid[agent_id]
            else:
                # Only fall back to agent_pos if we've never computed a valid centroid
                centroid = agent_pos
                self.last_valid_centroid[agent_id] = centroid
        
        # Cache result
        self.centroid_cache[agent_id] = (current_fingerprint, centroid, self.current_time)
        
        return centroid

    def compute_hyperbola_points(self, p_i, r_i, p_j, r_j, guaranteed=True):
        """Compute points on the hyperbola boundary between two uncertain regions"""
        if self.world_size is None:
            return []
        
        p_i, p_j = np.array(p_i), np.array(p_j)
        
        if guaranteed:
            k = -(r_i + r_j)
        else:
            k = r_i + r_j
        
        dist = np.linalg.norm(p_j - p_i)
        if abs(k) >= dist or dist == 0:
            return []
        
        c = dist / 2
        a = abs(k) / 2
        
        if a >= c or a <= 0:
            return []
            
        b = np.sqrt(c*c - a*a)
        
        center = (p_i + p_j) / 2
        direction = (p_j - p_i) / dist
        perpendicular = np.array([-direction[1], direction[0]])
        
        points = []
        t_values = np.linspace(-3, 3, 60)
        
        for t in t_values:
            if k < 0:
                x = -a * np.cosh(t)
            else:
                x = a * np.cosh(t)
            
            y = b * np.sinh(t)
            point = center + x * direction + y * perpendicular
            
            if (0 <= point[0] <= self.world_size[0] and 
                0 <= point[1] <= self.world_size[1]):
                points.append(point)
        
        return points
    
    def get_voronoi_neighbors(self):
        """Extract Voronoi neighbor relationships"""
        if not hasattr(self, 'vor') or self.vor == []:
            return {}
        return self.get_voronoi_neighbors_from_vor(self.vor)
    
    def get_voronoi_neighbors_from_vor(self, vor):
        """Extract Voronoi neighbor relationships from a given diagram"""
        if vor == []:
            return {}
        
        neighbors = {}
        
        for agent in self.population:
            agent_id = int(agent.name)
            neighbors[agent_id] = set()
        
        for ridge_points in vor.ridge_points:
            point_idx_i, point_idx_j = ridge_points
            
            if point_idx_i < len(vor.filtered_points) and point_idx_j < len(vor.filtered_points):
                pos_i = vor.filtered_points[point_idx_i]
                pos_j = vor.filtered_points[point_idx_j]
                
                agent_id_i = None
                agent_id_j = None
                
                for agent in self.population:
                    agent_pos = agent.getPosition()
                    agent_id = int(agent.name)
                    
                    if np.allclose(agent_pos, pos_i, atol=1e-6):
                        agent_id_i = agent_id
                    if np.allclose(agent_pos, pos_j, atol=1e-6):
                        agent_id_j = agent_id
                
                if agent_id_i is not None and agent_id_j is not None:
                    neighbors[agent_id_i].add(agent_id_j)
                    neighbors[agent_id_j].add(agent_id_i)
        
        return {k: list(v) for k, v in neighbors.items()}

    def compute_guaranteed_dual_boundaries_single_agent(self, agent_id, neighbor_ids):
        """Compute hyperbolic boundaries ONLY for the specified agent"""
        if len(self.population) < 2:
            return [], []
        
        if self.use_dynamic_uncertainty and agent_id in self.agent_memory:
            pos_i = np.array(self.agent_memory[agent_id]['last_known_pos'])
            r_i = self.agent_memory[agent_id]['uncertainty_radius']
        else:
            for agent in self.population:
                if int(agent.name) == agent_id:
                    pos_i = np.array(agent.getPosition())
                    r_i = self.fixed_uncertainty_radius
                    break
        
        guaranteed_segments = []
        dual_guaranteed_segments = []
        
        for neighbor_id in neighbor_ids:
            if self.use_dynamic_uncertainty and neighbor_id in self.agent_memory:
                pos_j = np.array(self.agent_memory[neighbor_id]['last_known_pos'])
                r_j = self.agent_memory[neighbor_id]['uncertainty_radius']
            else:
                for agent in self.population:
                    if int(agent.name) == neighbor_id:
                        pos_j = np.array(agent.getPosition())
                        r_j = self.fixed_uncertainty_radius
                        break
            
            guaranteed_points = self.compute_hyperbola_points(
                pos_i, r_i, pos_j, r_j, guaranteed=True
            )
            
            if guaranteed_points:
                guaranteed_segments.append({
                    'neighbor_id': neighbor_id,
                    'neighbor_pos': pos_j,
                    'points': guaranteed_points
                })
            
            dual_guaranteed_points = self.compute_hyperbola_points(
                pos_i, r_i, pos_j, r_j, guaranteed=False
            )
            
            if dual_guaranteed_points:
                dual_guaranteed_segments.append({
                    'neighbor_id': neighbor_id,
                    'neighbor_pos': pos_j,
                    'points': dual_guaranteed_points
                })
        
        return guaranteed_segments, dual_guaranteed_segments

    def compute_guaranteed_dual_boundaries(self, voronoi_neighbors=None):
        """
        Compute hyperbolic boundaries for ALL agents (for visualization only!)
        
        IMPORTANT: This uses REAL current positions from self.population for visualization,
        not per-agent memory (which only contains active set). This is a "god view" for
        debugging/visualization purposes - agents don't have this global information.
        """
        if len(self.population) < 2:
            return
            
        self.guaranteed_boundaries = []
        self.dual_guaranteed_boundaries = []
        self.guaranteed_centroids = {}
        
        # VISUALIZATION FIX: Build complete agent list from real positions
        agents = []
        for pop_agent in self.population:
            agent_id = int(pop_agent.name)
            pos = np.array(pop_agent.getPosition())
            
            # Get uncertainty from memory if available (for agents in our active set)
            # Otherwise use 0 (we don't know their uncertainty)
            if self.use_dynamic_uncertainty and agent_id in self.agent_memory:
                r = self.agent_memory[agent_id]['uncertainty_radius']
            else:
                r = 0.0 if self.use_dynamic_uncertainty else self.fixed_uncertainty_radius
            
            agents.append((agent_id, pos, r))
        
        agent_dict = {agent_id: (pos, r) for agent_id, pos, r in agents}
        
        # Determine neighbor relationships
        if voronoi_neighbors is None:
            voronoi_neighbors = {}
            
            # For OUR agent (the one this controller controls), use our active set
            if self.agent:
                our_agent_id = int(self.agent.name)
                voronoi_neighbors[our_agent_id] = list(self.active_set - {our_agent_id})
            
            # For all other agents, we don't know their active sets,
            # so use regular Voronoi neighbors as approximation for visualization
            vor, _ = self.compute_voronoi_once_per_frame()
            if vor != []:
                vor_neighbors = self.get_voronoi_neighbors_from_vor(vor)
                for agent_id in vor_neighbors:
                    if agent_id not in voronoi_neighbors:
                        voronoi_neighbors[agent_id] = vor_neighbors[agent_id]
        
        for agent_i_id, pos_i, r_i in agents:
            agent_guaranteed_segments = []
            agent_dual_guaranteed_segments = []
            
            neighbor_ids = voronoi_neighbors.get(agent_i_id, [])
            
            if self.use_dynamic_uncertainty:
                guaranteed_centroid = self.compute_guaranteed_voronoi_centroid_fast(agent_i_id, pos_i)
                self.guaranteed_centroids[agent_i_id] = guaranteed_centroid
            
            for neighbor_id in neighbor_ids:
                if neighbor_id not in agent_dict:
                    continue
                    
                pos_j, r_j = agent_dict[neighbor_id]
                
                guaranteed_points = self.compute_hyperbola_points(
                    pos_i, r_i, pos_j, r_j, guaranteed=True
                )
                
                if guaranteed_points:
                    agent_guaranteed_segments.append({
                        'neighbor_id': neighbor_id,
                        'neighbor_pos': pos_j,
                        'points': guaranteed_points
                    })
                
                dual_guaranteed_points = self.compute_hyperbola_points(
                    pos_i, r_i, pos_j, r_j, guaranteed=False
                )
                
                if dual_guaranteed_points:
                    agent_dual_guaranteed_segments.append({
                        'neighbor_id': neighbor_id,
                        'neighbor_pos': pos_j,
                        'points': dual_guaranteed_points
                    })
            
            if agent_guaranteed_segments:
                self.guaranteed_boundaries.append({
                    'agent_id': agent_i_id,
                    'agent_pos': pos_i,
                    'segments': agent_guaranteed_segments
                })
            
            if agent_dual_guaranteed_segments:
                self.dual_guaranteed_boundaries.append({
                    'agent_id': agent_i_id,
                    'agent_pos': pos_i,
                    'segments': agent_dual_guaranteed_segments
                })

    def compute_voronoi_once_per_frame(self):
        """Compute Voronoi diagram once per frame, cached (for visualization)"""
        if self.world_size is None or self.population is None:
            return [], {}
        
        if self.voronoi_cache_time == self.current_time and self.cached_vor is not None:
            return self.cached_vor, self.cached_centroids
        
        bounding_box = np.array([0.0, self.world_size[0], 0.0, self.world_size[0]])
        
        agent_positions = []
        position_to_agent_id = {}
        
        for pop_agent in self.population:
            pos = pop_agent.getPosition()
            agent_id = int(pop_agent.name)
            agent_positions.append(pos)
            position_to_agent_id[tuple(pos)] = agent_id
        
        points = np.array(agent_positions)
        vor = self.voronoi(points, bounding_box)
        
        centroids_dict = {}
        if vor != []:
            for i, region in enumerate(vor.filtered_regions):
                vertices = vor.vertices[region + [region[0]], :]
                centroid = self.centroid_region(vertices)
                
                region_center = np.mean(vor.vertices[region], axis=0)
                distances = [np.linalg.norm(region_center - point) for point in vor.filtered_points]
                closest_point_idx = np.argmin(distances)
                closest_point = tuple(vor.filtered_points[closest_point_idx])
                
                if closest_point in position_to_agent_id:
                    region_agent_id = position_to_agent_id[closest_point]
                    centroids_dict[region_agent_id] = np.array(centroid[0, :])
        
        self.voronoi_cache_time = self.current_time
        self.cached_vor = vor
        self.cached_centroids = centroids_dict
        
        return vor, centroids_dict

    def get_actions(self, agent):
        """
        PAPER FAITHFUL: Implements Table 5 (self-triggered centroid algorithm)
        
        Lines 1-5: Update memory, check condition, optionally update information
        Lines 6-8: Compute L, U, q, r with CURRENT D
        Line 11: Move to tbb(pi, vmax*dt, q, r)
        Line 12: Update D^i_i with future position
        """
        if not self.world or not hasattr(self.world, 'population') or self.world_size is None:
            return 0, 0
        
        current_agent_id = int(agent.name)
        current_pos = np.array(agent.getPosition())

        if current_agent_id == 0 and self.use_dynamic_uncertainty:
            self.maybe_record_H()
        
        if self.use_dynamic_uncertainty:
            # Paper's Table 5, Lines 1-2: set D = D^i (done in memory)
            self.update_agent_memory(agent)
            
            # Paper's Table 5, Lines 3-5: Check condition and optionally update
            self.trigger_information_update(agent)
            
            # Paper's Table 5, Lines 6-8: Compute L, U, q, r with CURRENT D
            # CRITICAL: Always compute fresh with current uncertainty state
            goal_position = self.compute_guaranteed_voronoi_centroid_fast(
                current_agent_id, current_pos
            )
            r = self.compute_bnd_approximate(current_agent_id)
        else:
            # Non-dynamic case: use regular Voronoi
            vor, centroids_dict = self.compute_voronoi_once_per_frame()
            goal_position = centroids_dict.get(current_agent_id, None)
            if goal_position is None:
                return 0, 0
            r = 0.0
        
        # Store for visualization
        self.guaranteed_centroids[current_agent_id] = goal_position
        
        # Control logic
        if self.use_tbb_motion:
            # Paper's Table 5, Line 11: move to tbb(pi, vmax*dt, q, r)
            
            # Maximum movement distance
            delta = self.v_max * (self.world_dt if self.world_dt else 0.025)
            
            # Compute next position using tbb (Algorithm 1)
            next_pos = self.tbb(current_pos, delta, goal_position, r)
            
            # Convert to velocity and angular velocity
            movement = next_pos - current_pos
            dist_to_move = np.linalg.norm(movement)
            dist_to_goal = np.linalg.norm(goal_position - current_pos)
            
            # Stop when close enough to goal
            if dist_to_goal < self.goal_tolerance:
                # Even when stopped, update D^i_i
                if self.use_dynamic_uncertainty and current_agent_id in self.agent_memory:
                    self.agent_memory[current_agent_id]['last_known_pos'] = current_pos
                    self.agent_memory[current_agent_id]['last_update_time'] = self.current_time
                    self.agent_memory[current_agent_id]['uncertainty_radius'] = 0.0
                return 0, 0
            
            if dist_to_move > 1e-6:
                # Compute desired heading
                desired_angle = np.arctan2(movement[1], movement[0])
                angle_diff = desired_angle - agent.angle
                
                # Normalize angle
                while angle_diff > np.pi:
                    angle_diff -= 2 * np.pi
                while angle_diff < -np.pi:
                    angle_diff += 2 * np.pi
                
                # SMOOTH DECELERATION: Proportional velocity based on distance to goal
                base_v = dist_to_move / (self.world_dt if self.world_dt else 0.025)
                
                if dist_to_goal < self.deceleration_radius:
                    # Proportional deceleration: v = v_max * (dist / decel_radius)
                    decel_factor = dist_to_goal / self.deceleration_radius
                    v = base_v * decel_factor
                else:
                    v = base_v
                
                # Cap at max velocity and ensure minimum velocity when moving
                v = np.clip(v, 0.05, self.v_max)  # Minimum 0.05 to avoid getting stuck
                
                # Angular velocity
                if abs(angle_diff) > 0.1:
                    agent.angle = np.arctan2(goal_position[1] - agent.pos[1], goal_position[0] - agent.pos[0])
                    # Slow down linear velocity when turning sharply
                    if abs(angle_diff) > 0.5:
                        v *= 0.7  # Reduce speed by 30% during sharp turns
                else:
                    omega = 0
                
                # CRITICAL: Paper's Table 5, Line 12: set D^i_i = (tbb(...), 0)
                # Update memory with WHERE WE WILL BE, not where we are
                if self.use_dynamic_uncertainty and current_agent_id in self.agent_memory:
                    self.agent_memory[current_agent_id]['last_known_pos'] = next_pos
                    self.agent_memory[current_agent_id]['last_update_time'] = self.current_time
                    self.agent_memory[current_agent_id]['uncertainty_radius'] = 0.0
            else:
                v, omega = 0, 0
                # Update even when not moving
                if self.use_dynamic_uncertainty and current_agent_id in self.agent_memory:
                    self.agent_memory[current_agent_id]['last_known_pos'] = current_pos
                    self.agent_memory[current_agent_id]['last_update_time'] = self.current_time
                    self.agent_memory[current_agent_id]['uncertainty_radius'] = 0.0
        else:
            # Original proportional control with smooth deceleration
            v, omega = 0, 0
            dist_to_goal = np.linalg.norm(agent.pos - goal_position)
            
            # Stop when close enough
            if dist_to_goal < self.goal_tolerance:
                return 0, 0
            
            radians_to_goal = np.arctan2(goal_position[1] - agent.pos[1], goal_position[0] - agent.pos[0]) - agent.angle
            
            # Normalize angle
            while radians_to_goal > np.pi:
                radians_to_goal -= 2 * np.pi
            while radians_to_goal < -np.pi:
                radians_to_goal += 2 * np.pi

            
            if dist_to_goal > agent.radius:
                # SMOOTH DECELERATION: Proportional velocity
                if dist_to_goal < self.deceleration_radius:
                    # Slow down smoothly as approaching goal
                    decel_factor = dist_to_goal / self.deceleration_radius
                    v = self.v_max * decel_factor * np.clip(dist_to_goal, 0, 1)
                else:
                    # Normal speed when far from goal
                    v = 0.3 * np.clip(dist_to_goal, 0, 1)
                
                # Ensure minimum velocity
                v = max(v, 0.05) if dist_to_goal > self.goal_tolerance else 0
                
                if abs(radians_to_goal) > 0.1:
                    agent.angle = np.arctan2(goal_position[1] - agent.pos[1], goal_position[0] - agent.pos[0])
                    # Slow down during sharp turns
                    if abs(radians_to_goal) > 0.5:
                        v *= 0.7
                else:
                    omega = 0
        omega = 0
        return v, omega


    # TODO: outdated, based on much older code. Rewrite now that the code is accurate to the paper. kinda works but it sucks
    def draw(self, screen, offset):
        """Visualization - only draws when flags are enabled"""
        pan, zoom = np.asarray(offset[0]), offset[1]
        super().draw(screen, offset)

        if self.draw_uncertainty_circles and self.population:
            if self.use_dynamic_uncertainty:

                for pop_agent in self.population:
                    agent_id = int(pop_agent.name)
                    
                    if agent_id in self.agent_memory:
                        pos = self.agent_memory[agent_id]['last_known_pos']
                        radius = self.agent_memory[agent_id]['uncertainty_radius']
                        
                        if radius > 0.01:
                            pygame.draw.circle(screen, (255, 255, 0), (pos * zoom + pan).astype(int), 
                                            int(radius * zoom), width=2)
            
            elif self.show_mode in ["guaranteed", "dual_guaranteed", "all"]:
                for pop_agent in self.population:
                    pos = np.array(pop_agent.getPosition())
                    radius = self.fixed_uncertainty_radius
                    
                    if radius > 0:
                        pygame.draw.circle(screen, (255, 255, 0), (pos * zoom + pan).astype(int), 
                                        int(radius * zoom), width=2)

        need_voronoi = (self.draw_voronoi or 
                       self.show_mode in ["guaranteed", "dual_guaranteed", "all"])
        
        if need_voronoi:
            vor, centroids_dict = self.compute_voronoi_once_per_frame()
            
            if self.draw_voronoi and vor is not None and vor != []:
                allpairs = []
                centroids = []
                
                for i, region in enumerate(vor.filtered_regions):
                    vertices = vor.vertices[region + [region[0]], :]
                    centroid = self.centroid_region(vertices)
                    centroids.append(centroid[0, :])
                    
                    for j in range(1, len(vertices)):
                        allpairs.append([vertices[j-1], vertices[j]])
                
                for line in allpairs:
                    pygame.draw.line(screen, (128, 128, 128), 
                                    (np.array(line[0]) * zoom + pan).astype(int), 
                                    (np.array(line[1]) * zoom + pan).astype(int), width=2)
                
                for centroid in centroids:
                    pygame.draw.circle(screen, (255, 255, 255), 
                                        (np.array(centroid) * zoom + pan).astype(int), radius=4, width=2)
            
            if self.show_mode in ["guaranteed", "dual_guaranteed", "all"]:
                if (not hasattr(self, '_boundaries_computed_time') or 
                    self._boundaries_computed_time != self.current_time):
                    
                    if vor is not None and vor != []:
                        voronoi_neighbors = self.get_voronoi_neighbors_from_vor(vor)
                        self.compute_guaranteed_dual_boundaries(voronoi_neighbors)
                        self._boundaries_computed_time = self.current_time
        
        if self.use_dynamic_uncertainty and self.guaranteed_centroids:
            for agent_id, g_centroid in self.guaranteed_centroids.items():
                pygame.draw.circle(screen, (0, 255, 0), 
                                (g_centroid * zoom + pan).astype(int), radius=6, width=2)
        
        if self.show_mode in ["guaranteed", "all"]:
            colors = [(0, 255, 0), (255, 100, 0), (255, 0, 255), (0, 255, 255), (255, 255, 0), (100, 255, 100)]
            for agent_idx, agent_boundary in enumerate(self.guaranteed_boundaries):
                color = colors[agent_idx % len(colors)]
                agent_pos = agent_boundary['agent_pos']
                
                for segment in agent_boundary['segments']:
                    points = segment['points']
                    neighbor_pos = segment['neighbor_pos']
                    
                    points = self.trim_hyperbola_by_span(points, agent_pos, neighbor_pos)

                    if len(points) > 1:
                        screen_points = [(np.array(point) * zoom + pan).astype(int) for point in points]
                        for i in range(len(screen_points) - 1):
                            pygame.draw.line(screen, color, screen_points[i], screen_points[i + 1], width=3)

        if self.show_mode in ["dual_guaranteed", "all"]:
            colors = [(0, 100, 255), (255, 150, 100), (200, 100, 255), (100, 200, 255), (200, 200, 100), (150, 255, 150)]
            for agent_idx, agent_boundary in enumerate(self.dual_guaranteed_boundaries):
                color = colors[agent_idx % len(colors)]
                agent_pos = agent_boundary['agent_pos']

                for segment in agent_boundary['segments']:
                    points = segment['points']
                    neighbor_pos = segment['neighbor_pos']
                    
                    points = self.trim_hyperbola_by_span(points, agent_pos, neighbor_pos)
                    if len(points) > 1:
                        screen_points = [(np.array(point) * zoom + pan).astype(int) for point in points]
                        for i in range(len(screen_points) - 1):
                            pygame.draw.line(screen, color, screen_points[i], screen_points[i + 1], width=3)

    @staticmethod
    def in_box(towers, bounding_box):
        if towers.size == 0 or len(towers.shape) != 2 or towers.shape[1] < 2:
            return np.array([], dtype=bool)
        return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],
                                            towers[:, 0] <= bounding_box[1]),
                            np.logical_and(bounding_box[2] <= towers[:, 1],
                                            towers[:, 1] <= bounding_box[3]))

    @staticmethod
    def voronoi(towers, bounding_box):
        if towers.size == 0 or len(towers.shape) != 2 or towers.shape[1] < 2:
            return []
        
        eps = 0.01
        i = VoronoiController.in_box(towers, bounding_box)
        
        if not np.any(i):
            return []
        
        points_center = towers[i, :]
        points_left = np.copy(points_center)
        points_left[:, 0] = bounding_box[0] - (points_left[:, 0] - bounding_box[0])
        points_right = np.copy(points_center)
        points_right[:, 0] = bounding_box[1] + (bounding_box[1] - points_right[:, 0])
        points_down = np.copy(points_center)
        points_down[:, 1] = bounding_box[2] - (points_down[:, 1] - bounding_box[2])
        points_up = np.copy(points_center)
        points_up[:, 1] = bounding_box[3] + (bounding_box[3] - points_up[:, 1])
        points = np.append(points_center,
                        np.append(np.append(points_left,
                                            points_right,
                                            axis=0),
                                    np.append(points_down,
                                            points_up,
                                            axis=0),
                                    axis=0),
                        axis=0)
        
        if len(points) != 0:
            vor = spatial.Voronoi(points)
        else:
            return []
        
        regions = []
        for region in vor.regions:
            flag = True
            for index in region:
                if index == -1:
                    flag = False
                    break
                else:
                    x = vor.vertices[index, 0]
                    y = vor.vertices[index, 1]
                    if not(bounding_box[0] - eps <= x and x <= bounding_box[1] + eps and
                        bounding_box[2] - eps <= y and y <= bounding_box[3] + eps):
                        flag = False
                        break
            if region != [] and flag:
                regions.append(region)
        vor.filtered_points = points_center
        vor.filtered_regions = regions
        return vor
    
    @staticmethod
    def centroid_region(vertices):
        A = 0
        C_x = 0
        C_y = 0
        for i in range(0, len(vertices) - 1):
            s = (vertices[i, 0] * vertices[i + 1, 1] - vertices[i + 1, 0] * vertices[i, 1])
            A = A + s
            C_x = C_x + (vertices[i, 0] + vertices[i + 1, 0]) * s
            C_y = C_y + (vertices[i, 1] + vertices[i + 1, 1]) * s
        A = 0.5 * A
        if abs(A) < 1e-10:
            return np.array([[np.mean(vertices[:-1, 0]), np.mean(vertices[:-1, 1])]])
        C_x = (1.0 / (6.0 * A)) * C_x
        C_y = (1.0 / (6.0 * A)) * C_y
        return np.array([[C_x, C_y]])
    
    def trim_hyperbola_by_span(self, hyperbola_points, agent_pos, neighbor_pos):
        """Trim hyperbola from endpoints until span is acceptable"""
        if len(hyperbola_points) < 2:
            return hyperbola_points
        
        agent_to_neighbor_dist = np.linalg.norm(neighbor_pos - agent_pos)
        points = list(hyperbola_points)
        
        while len(points) > 2:
            first_to_last_dist = np.linalg.norm(np.array(points[-1]) - np.array(points[0]))
            
            if first_to_last_dist <= agent_to_neighbor_dist:
                break
            
            dist_to_first = np.linalg.norm(np.array(points[0]) - agent_pos)
            dist_to_last = np.linalg.norm(np.array(points[-1]) - agent_pos)
            
            if dist_to_first > dist_to_last:
                points.pop(0)
            else:
                points.pop()
        
        return points