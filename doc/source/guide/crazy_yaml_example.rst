.. code-block:: yaml
   :caption: world.yaml (example)

   type: "RectangularWorld"
   size: [10, 10]  # yaml flow style
   agents:
   - !include robot1.yaml  # add robot1 agent
   - !include robot2.yaml  # add robot2 agent
   - &anchor003  # save this robot as an anchor
     type: MazeAgent
     name: robot3
     agent_radius: 0.1
     angle: !np radians(90 + 45)  # convert degrees to radians
     poly: !include body_shape.svg  # load an SVG file
     controller:
       type: StaticController
       output: !np [1e-2, pi / 2]  # pi constant from numpy
   spawners:
   - type: ExcelSpawner
     path: !relpath positions.xlsx  # path is relative to cwd or this YAML file
     agent: *anchor003  # use the robot3 agent from above