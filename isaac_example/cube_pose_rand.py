import sys
import pathlib
import cspace.torch.classes
import cspace.cspace.classes
import random
import torch


description = pathlib.Path(sys.argv[1]).read_text()

kinematics = cspace.cspace.classes.Kinematics(description, "panda_hand", base=None)
#print("JOINT: ", kinematics.joint)
#print("JOINT: ", len(kinematics.joint))
for i in range(1024):
  scale = torch.rand(len(kinematics.joint))
  state = cspace.torch.classes.JointStateCollection.apply(
                    kinematics.spec,
                    kinematics.joint,
                    scale,
                    min=0.0,
                    max=1.0,
                )
  pose = kinematics.forward(state)

  pp = list(pose.position(link).tolist() for link in kinematics.link)[0]
  print(pp[0], ',' , pp[1], ', ' , pp[2])
  #print(list(pose.position(link).tolist() for link in kinematics.link)[0])
