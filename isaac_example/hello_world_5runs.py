import random
import omni.usd
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core.tasks import BaseTask
import numpy as np
from omni.isaac.core.utils.types import ArticulationAction
#import omni.isaac.core.utils.types.ArticulationAction as ArticulationAction
#omni.isaac.core.utils.types.ArticulationAction
class FrankaPlaying(BaseTask):
    #NOTE: we only cover here a subset of the task functions that are available,
    # checkout the base class for all the available functions to override.
    # ex: calculate_metrics, is_done..etc.
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])
        self._task_achieved = False
        return

    # Here we setup all the assets that we care about in this task.
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        self._cube = scene.add(DynamicCuboid(prim_path="/World/random_cube",
                                            name="fancy_cube",
                                            position=np.array([0.3, 0.3, 0.3]),
                                            scale=np.array([0.0415, 0.0415, 0.0415]),
                                            color=np.array([0, 0, 1.0])))
        self._franka = scene.add(Franka(prim_path="/World/Fancy_Franka",
                                        name="fancy_franka"))
        return

    # Information exposed to solve the task is returned from the task through get_observations
    def get_observations(self):
        cube_position, _ = self._cube.get_world_pose()
        current_joint_positions = self._franka.get_joint_positions()
        observations = {
            self._franka.name: {
                "joint_positions": current_joint_positions,
            },
            self._cube.name: {
                "position": cube_position,
                "goal_position": self._goal_position
            }
        }
        return observations

    # Called before each physics step,
    # for instance we can check here if the task was accomplished by
    # changing the color of the cube once its accomplished
    def pre_step(self, control_index, simulation_time):
        cube_position, _ = self._cube.get_world_pose()
        if not self._task_achieved and np.mean(np.abs(self._goal_position - cube_position)) < 0.02:
            # Visual Materials are applied by default to the cube
            # in this case the cube has a visual material of type
            # PreviewSurface, we can set its color once the target is reached.
            self._cube.get_applied_visual_material().set_color(color=np.array([0, 1.0, 0]))
            self._task_achieved = True
        return


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.runnum=5;
        self.franka_prim =None
        return

    def setup_scene(self):
        #this only called once when plugin code load from menu. 
        #if modify code and reload from, it will not 
        #to be sure, New to clear up, then load from isaac example menu
        world = self.get_world()
        # We add the task to the world here
        self.franka_play = FrankaPlaying(name="my_first_task")
        world.add_task(self.franka_play)
        print('franka_play ', self.franka_play)
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        stage = omni.usd.get_context().get_stage()
        primrange= stage.Traverse()
        print('prim range ', primrange)
        for prim0 in primrange:
            if prim0.GetPath()=='/OmniverseKit_Persp':
                print('persp prm, ', prim0.GetAttribute('focalLength').Get())
                prim0.GetAttribute('focalLength').Set(100)
                print('persp prm, ', prim0.GetAttribute('focalLength').Get())
                self.persp_prim =prim0
            if prim0.GetPath()=='/World/Fancy_Franka':
                self.franka_prim = prim0
                print('franka prim ', prim0)
            if prim0.GetPath()=='/World/random_cube':
                self.target_prim = prim0
                print('target prim ', prim0)

            #print('prim ', prim0.GetPath())

        # The world already called the setup_scene from the task (with first reset of the world)
        # so we can retrieve the task objects
        self._franka = self._world.scene.get_object("fancy_franka")
        self._mycube = self._world.scene.get_object("fancy_cube")
        self._controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
        )
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        observations = self._world.get_observations()
        actions = self._controller.forward(
            picking_position=observations["fancy_cube"]["position"],
            placing_position=observations["fancy_cube"]["goal_position"],
            current_joint_positions=observations["fancy_franka"]["joint_positions"],
            )
        print('first action ', type(actions))
        #actions['joint_positions']=[None, None, None, None, None, None, None, 0.025724807009100917, -0.0222918625921011]
        #actions['joint_velocities']=None
        actions = ArticulationAction(joint_positions=np.array([0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.04, 0.04]))
        print('first action2 ', actions)
        self._franka.apply_action(actions)
    
        await self._world.play_async()
        return

    def nextsession(self):
        if self.runnum==0:
            return
        print('start next session, runnum: ', self.runnum)
        print('persp prm, ', self.persp_prim.GetAttribute('focalLength').Get())
        from omni.isaac.core.utils.viewports import set_camera_view
        camera_position=[-3.0 + random.random() * 8.0, -3.0 + random.random() * 6.0, 2.0 + random.random() * 4.0]
        from pxr import Gf
        import omni.usd
        matrix: Gf.Matrix4d = omni.usd.get_world_transform_matrix(self.franka_prim)
        target_translate: Gf.Vec3d = matrix.ExtractTranslation()

        #set camera view
        #set_camera_view(eye=camera_position, target=target_translate, camera_prim_path="/OmniverseKit_Persp")

        # change target_prim location
        from pxr import Gf, UsdGeom
        from omni.isaac.dynamic_control import _dynamic_control
        dc = _dynamic_control.acquire_dynamic_control_interface()
        mycube = dc.get_rigid_body("/World/random_cube")
        #print('cube handle ', mycube)
        matrix_t: Gf.Matrix4d = omni.usd.get_world_transform_matrix(self.target_prim)
        target_t: Gf.Vec3d = matrix_t.ExtractTranslation()
        x_r = random.random()*0.2 - 0.2
        y_r = random.random()*0.2 - 0.2 
        x_sign = np.sign(random.random()-0.5)
        y_sign = np.sign(random.random()-0.5)

        new_location_np = np.array([x_sign*(target_t[0]+x_r), y_sign*(target_t[1]+y_r), target_t[2]])
        new_location = Gf.Vec3d(target_t[0]+x_r, target_t[1]+y_r, target_t[2])
        new_rotation = Gf.Rotation(Gf.Vec3d(1, 0, 0), 0)
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(new_location)
        mat.SetRotateOnly(new_rotation)
        #dc.set_rigid_body_pose(mycube, dctf) #shitty api not work, no one use seems
        #self.franka_play._cube.set_world_pose(position=new_location_np, orientation=np.array([1., 0., 0., 0.]))

        #self._mycube.set_world_pose(position=new_location_np, orientation=np.array([1., 0., 0., 0.]))
        newcube = self._world.scene.add(DynamicCuboid(prim_path="/World/random_cube",
                                            name="fancy_cube"+str(x_r),
                                            position=new_location_np,
                                            scale=np.array([0.0415, 0.0415, 0.0415]),
                                            color=np.array([0, 0, 1.0])))

        current_observations = self._world.get_observations()
        print('obs ', current_observations)
        print('cube pose ', current_observations["fancy_cube"]["position"])
        print('cube goal posie ', current_observations["fancy_cube"]["goal_position"])

        self.runnum = self.runnum -1
        self._controller.reset()
        actions = ArticulationAction(joint_positions=np.array([None, None, None, None, None, None, None, 0.04, 0.04]))
        print(' action reset ', actions)
        self._franka.apply_action(actions)
        return

    async def setup_post_reset(self):
        self.runnum=5;
        self.startnewsession = True;
        while True:
            if self.runnum==0:
                break;
            if self.startnewsession:
                print('start new session, runnum: ', self.runnum)
                self.runnum = self.runnum -1
                self._controller.reset()
                actions = ArticulationAction(joint_positions=np.array([None, None, None, None, None, None, None, 0.04, 0.04]))
                print('first action reset ', actions)
                self._franka.apply_action(actions)
                #self._world.play()
                await self._world.play_async()
                self.startnewsession = False
                return
            #time.sleep(20)
        #await self._world.play_async()
        return

    def physics_step(self, step_size):
        # Gets all the tasks observations
        current_observations = self._world.get_observations()
        actions = self._controller.forward(
            picking_position=current_observations["fancy_cube"]["position"],
            placing_position=current_observations["fancy_cube"]["goal_position"],
            current_joint_positions=current_observations["fancy_franka"]["joint_positions"],
        )
        #print('action ', actions)
        self._franka.apply_action(actions)
        if self._controller.is_done():
            self.startnewsession = True;
            self.nextsession()
            if self.runnum==0:
                self._world.pause()
        return
