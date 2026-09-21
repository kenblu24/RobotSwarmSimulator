from swarmsim import config_from_yaml, run_sim


world_cfg = config_from_yaml("demo/configs/boids_conf/world.yaml")
run_sim(world_cfg)

# Prevent program from just exiting
input()