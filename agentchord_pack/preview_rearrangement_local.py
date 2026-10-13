import runpy
from embodichain.lab.gym.envs.tasks.tableware.rearrangement import RearrangementEnv

original_check = RearrangementEnv.is_task_success

def preview_check(self):
    plate_pose = self.sim.get_rigid_object("plate").get_local_pose(to_matrix=True)[0]
    for name, offset in (("spoon", -0.16), ("fork", 0.16)):
        target = plate_pose.clone()
        target[1, 3] += offset
        self.affordance_datas.setdefault(f"{name}_place_pose", target)
    return original_check(self)

RearrangementEnv.is_task_success = preview_check
runpy.run_path("embodichain/lab/scripts/run_env.py", run_name="__main__")
