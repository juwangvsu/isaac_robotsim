import numpy as np
from omni.isaac.core import World
import time
from omni.replicator.core import AnnotatorRegistry, BackendDispatch, Writer, WriterRegistry
import omni.syntheticdata.scripts.helpers
import json
panda_joint_prim =['/World/Fancy_Franka/panda_link0/panda_joint1']
class MyCustomWriter(Writer):
    def __init__(
        self,
        output_dir,
        rgb = True,
        normals = False,
    ):
        world= World()
        self.worldins = world.instance()
        print('get world instance ', self.worldins)
        print('init mycustomwrite: ', rgb, normals, self.version)
        print('init mycustomwrite: get_pose', self.get_pose())
        '''
        mycustomwrite: get_pose [('/World/random_cube', 1, 'cube', array([[0.0515    , 0.        , 0.        , 0.        ],
       [0.        , 0.0515    , 0.        , 0.        ],
       [0.        , 0.        , 0.0515    , 0.        ],
       [0.29999673, 0.3000026 , 0.02574998, 1.        ]]))]
       '''
        self.version = "0.0.1"
        self.backend = BackendDispatch({"paths": {"out_dir": output_dir}})
        if rgb:
            self.annotators.append(AnnotatorRegistry.get_annotator("rgb"))
        if normals:
            self.annotators.append(AnnotatorRegistry.get_annotator("normals"))
        self._frame_id = 0

    def write(self, data: dict):
        self._frame_id += 1
        tstamp=time.time()
        if not self._frame_id % 10 ==0:
           return
        obj_poses = self.get_pose() #for target objs, use semantic editor to assign semantic label, list of tr matrix
        print('obj_poses len ', len(obj_poses))

        #get_pose return array
        #might have two types of stuff: obj position, and joint values
        #of robot. the former is always 4-tuples, the later is 3 tuple and index[1] is 'joint_position'
        #joint_position item is alway last in the array
        lastid = len(obj_poses)-1
        if len(obj_poses)>0 and not obj_poses[0][1]=='joint_position':
            #this is a object position
            print('xxxx ', obj_poses[lastid])
            pose_np = obj_poses[0][3].tolist()
            filename0 = f"{self.backend.output_dir}/pose_{self._frame_id:06d}_{tstamp}.json"
            with open(filename0, "w") as f:
                print("\n\n\n:json dumping pose_np : ", filename0)
                json.dump(pose_np, f)
        else:
            print('no object, add semantic label to target object if desired')

        #check if we get the joint position
        print('xxxx ', obj_poses[lastid])
        if len(obj_poses)>0 and obj_poses[lastid][1]=='joint_position' and not obj_poses[lastid][2] is None:
            jp_np = obj_poses[lastid][2].tolist()
            jn_np = obj_poses[lastid][3]
            filename0 = f"{self.backend.output_dir}/joint_{self._frame_id:06d}_{tstamp}.json"
            with open(filename0, "w") as f:
                print("\n\n\n:json dumping jp_np : ", filename0)
                json.dump(dict(zip(jn_np, jp_np)), f)
        
        print('mycustomwrite: get_pose', obj_poses)

        for annotator in data.keys():
            # If there are multiple render products the data will be stored in subfolders
            annotator_split = annotator.split("-")
            render_product_path = ""
            multi_render_prod = 0
            if len(annotator_split) > 1:
                multi_render_prod = 1
                render_product_name = annotator_split[-1]
                render_product_path = f"{render_product_name}/"

            # rgb
            if annotator.startswith("rgb"):
                if multi_render_prod:
                    render_product_path += "rgb/"
                filename = f"{render_product_path}joint_{self._frame_id:06d}_{tstamp}.png"
                print(f"[{self._frame_id}] Writing {self.backend.output_dir}/{filename} ..")
                self.backend.write_image(filename, data[annotator] )

            # semantic_segmentation
            if annotator.startswith("normals"):
                if multi_render_prod:
                    render_product_path += "normals/"
                filename = f"{render_product_path}normals_{self._frame_id:06d}_{tstamp}.png"
                print(f"[{self._frame_id}] Writing {self.backend.output_dir}/{filename} ..")
                colored_data = ((data[annotator] * 0.5 + 0.5) * 255).astype(np.uint8)
                self.backend.write_image(filename, colored_data )


    def on_final_frame(self):
        self._frame_id = 0

    def get_pose(self, viewport=None):
        """Get pose of all objects with a semantic label.
        """
        stage = omni.usd.get_context().get_stage()
        mappings = omni.syntheticdata.scripts.helpers.get_instance_mappings()
        pose = []
        pj1_prim = stage.GetPrimAtPath(panda_joint_prim[0])
        franka_prim = stage.GetPrimAtPath("/World/Fancy_Franka")
        print('franka ', franka_prim)
        franka_obj = self.worldins.scene.get_object("fancy_franka")
        print('franka_obj ', franka_obj)
        if not franka_obj is None:
            joint_pos = franka_obj.get_joint_positions()
            joint_pos2 = franka_obj.get_joint_positions(joint_indices=np.array([7, 8]))
            #prim.get_joint_positions(joint_indices=np.array([7, 8]))
            #print("DOF names:", franka_obj.dof_names)
            joint_names = franka_obj.dof_names
            #j1_ind = franka_prim.get_joint_index("panda_joint1")
            print('franka_obj joint positions ', joint_pos)
            #print('franka_obj joint index ', j1_ind)

        pj1_atts = pj1_prim.GetPropertyNames()
        pj1_att = next(
            (att_name for att_name in pj1_prim.GetPropertyNames()
                if att_name.endswith(":physics:position")), None)
        print('pj1 ', pj1_prim, pj1_atts, pj1_att)
        for m in mappings:
            prim_path = m[1]
            prim = stage.GetPrimAtPath(prim_path)
            prim_tf = omni.usd.get_world_transform_matrix(prim)
            translation = prim_tf.ExtractTranslation()
            rotation = prim_tf.ExtractRotation()
            print('trans ', translation)
            #print('rotation ', rotation)
            rotmat=np.array(prim_tf)[0:3,0:3]
            eyemat = np.matmul(rotmat, rotmat.T)
            #print('eye assert: ', eyemat, rotmat)
            pose.append((str(prim_path), m[2], str(m[3]), np.array(prim_tf)))
        if not franka_obj is None:
            pose.append(('fancy_frank', 'joint_position', joint_pos, joint_names))
        return pose
WriterRegistry.register(MyCustomWriter)
