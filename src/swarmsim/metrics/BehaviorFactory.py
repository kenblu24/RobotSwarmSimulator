from ..metrics.AgentsAtGoal import AgentsAtGoal, PercentageAtGoal
from ..metrics.AverageSpeed import AverageSpeedBehavior
from ..metrics.AlgebraicConnectivity import AlgebraicConn
from ..metrics.ScatterBehavior import ScatterBehavior
from ..metrics.DistanceToGoal import DistanceToGoal
from ..metrics.TotalCollisions import TotalCollisionsBehavior
from ..metrics.AngularMomentum import AngularMomentumBehavior
from ..metrics.Centroid import Centroid
from ..metrics.ConvexHull import ConvexHull
from ..metrics.DistanceToGoal import DistanceToGoal
from ..metrics.GroupRotationBehavior import GroupRotationBehavior
from ..metrics.RadialVariance import RadialVarianceMetric
from ..metrics.SensorOffset import GeneElementDifference
from ..metrics.SensorRotation import SensorRotation
from ..metrics.SensorSignal import SensorSignalBehavior

class BehaviorFactory:
    @staticmethod
    def create(d):
        if d["name"] == "Goal_Agents":
            return AgentsAtGoal(history=d["history_size"])
        elif d["name"] == "Alg_Connectivity":
            return AlgebraicConn(history=d["history_size"])
        elif d["name"] == "Angular_Momentum":
            return AngularMomentumBehavior(history=d["history_size"])
        elif d["name"] == "Average_Speed":
            return AverageSpeedBehavior(history=d["history_size"])
        elif d["name"] == "Centroid":
            return Centroid(history=d["history_size"])
        elif d["name"] == "Convex_Hull_Area":
            return ConvexHull(history=d["history_size"])
        elif d["name"] == "Goal_Dist":
            return DistanceToGoal(history=d["history_size"])
        elif d["name"] == "Group_Rotation":
            return GroupRotationBehavior(history=d["history_size"])
        elif d["name"] == "Radial_Variance":
            return RadialVarianceMetric(history=d["history_size"])
        elif d["name"] == "Scatter":
            return ScatterBehavior(history=d["history_size"])
        # elif d["name"] == "Sensor_Offset":
        #     return GeneElementDifference(history=d["history_size"])
        # elif d["name"] == "Sensor_Rotation":
        #     return SensorRotation(history=d["history_size"])
        elif d["name"] == "Total_Collisions":
            return TotalCollisionsBehavior(history=d["history_size"])
        elif d["name"].endswith("at_goal"):
            return PercentageAtGoal(history=d["history_size"], percentage=d["percentage"])
        else:
            raise Exception(f"Cannot Construct Behavior of Type {d['name']}")
