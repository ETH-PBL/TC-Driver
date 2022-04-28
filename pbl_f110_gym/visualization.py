from stable_baselines3.common.callbacks import BaseCallback, EventCallback
from typing import Optional
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from copy import copy

from splinify import SplineTrack 

class SaveObservations(EventCallback):
    """
    A custom callback that derives from ``BaseCallback``.

    :param verbose: (int) Verbosity level 0: not output 1: info 2: debug
    """
    def __init__(self, map_path, visualize_callback: Optional[BaseCallback] = None, verbose=0):
        super(SaveObservations, self).__init__(visualize_callback, verbose=verbose)

        self.map_path = map_path
        track = SplineTrack(self.map_path)

        self.x_out, self.y_out = track.outline_x(np.arange(0, track.track_length)), track.outline_y(np.arange(0, track.track_length))
        self.x_mid, self.y_mid = track.midline_x(np.arange(0, track.track_length)), track.midline_y(np.arange(0, track.track_length))
        self.x_int, self.y_int = track.intline_x(np.arange(0, track.track_length)), track.intline_y(np.arange(0, track.track_length))

        self.line_out = matplotlib.lines.Line2D(self.x_out, self.y_out)
        self.line_mid = matplotlib.lines.Line2D(self.x_mid, self.y_mid)
        self.line_int = matplotlib.lines.Line2D(self.x_int, self.y_int)

        self.env = None
        self.obs_dict = None
        self.obs_dicts = []

    def _on_rollout_end(self) -> None:
        self.env = self.training_env.venv.unwrapped.envs[0].env.env.env

        if (self.env.is_reset):
            self.obs_dict = self.env.last_obs_dict
            self.obs_dicts.append(self.obs_dict)

            self._on_event()

class VisualizePosition(BaseCallback):
    def __init__(self):
        super().__init__()
        
        self.num_images = 0

        self.fig = plt.figure(frameon=False)

    def _on_step(self) -> bool:
        assert self.parent is not None, "``VisualizePosition`` callback must be used " "with an ``EvalCallback``"

        self.plot_position_episode()
        self.plot_position_track()

        plt.close(111)

        return True

    def plot_position_episode(self):
        ax = self.fig.add_subplot(111)
        ax.axis('off')

        ax.add_line(copy(self.parent.line_out))
        ax.add_line(copy(self.parent.line_int))

        ax.set_xlim(min(self.parent.x_out - 5), max(self.parent.x_out + 5))
        ax.set_ylim(min(self.parent.y_out - 5), max(self.parent.y_out + 5))

        car_positions = np.array(self.parent.obs_dict['car_position'][1:-1])

        line_run = matplotlib.lines.Line2D(car_positions[:, 0] , car_positions[:, 1])
        ax.add_line(line_run)

        self.save_track_image(f'images/plot_car_position_episode_{self.num_images}.svg')

        self.num_images += 1

    def plot_position_track(self):
        ax = self.fig.add_subplot(111)
        ax.axis('off')
        
        ax.add_line(copy(self.parent.line_out))
        ax.add_line(copy(self.parent.line_int))

        ax.set_xlim(min(self.parent.x_out - 5), max(self.parent.x_out + 5))
        ax.set_ylim(min(self.parent.y_out - 5), max(self.parent.y_out + 5))

        for obs_dict in self.parent.obs_dicts:
            car_positions = np.array(obs_dict['car_position'][1:-1])

            line_run = matplotlib.lines.Line2D(car_positions[:, 0] , car_positions[:, 1])
            ax.add_line(line_run)

        self.save_track_image(f'images/plot_car_position_track.svg')

    def save_track_image(self, fname):
        plt.savefig(fname, bbox_inches='tight')
        plt.close(111)