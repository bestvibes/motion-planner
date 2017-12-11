import mujoco_py
import time
import click
import pickle

class Viz_Trajectory:
    def __init__(self, model_path):
        self.model = mujoco_py.load_model_from_xml(model)
        self.sim = mujoco_py.MjSim(self.model)
        self.viewer = mujoco_py.MjViewer(self.sim)
        self.qdim = self.model.nq
        self.udim = self.model.nu
        self.pdim = 2*self.qdim + self.udim


    def draw_frame(self, point, dt):
        q = point[:self.qdim]
        v = point[self.qdim:2*self.qdim]
        self.sim.data.qpos[:] = q
        self.sim.data.qvel[:] = v
        self.viewer.render()
        time.sleep(dt)


    def __call__(self, traj, dt):
        assert (traj.shape)<3
        if (traj.shape)==1:
            traj_2d = traj.reshape(-1, self.pdim)
        else:
            traj_2d = traj
        for point in traj_2d:
            self.draw_frame(point, dt)
        return True

@click.command()
@click.option('--scenario', type=str, default='', help='name of scenario script')
@click.option('--traj_filename', type=str, default='/tmp/trajectory.pkl', help='filename of the solution trajectory')
def main(scenario, solution_filename):
    viz = viz_Trajectory(scenario)
    traj = pickle.load(traj_filename)
    viz(traj, 5.0/20)


if __name__ == "__main__":
    main()


