import time

import jsonlines
import numpy as np
import pyglet
from f110_gym.envs.rendering import EnvRenderer

WINDOW_W = 1000
WINDOW_H = 800
MAP_PATH = "/Users/kubilayeksioglu/Works/acrome/experiments/f1tenth-riders-evaluator/f1tenth_gym_ros/maps/SILVERSTONE"
LOG_PATH = "/Users/kubilayeksioglu/Works/acrome/experiments/f1tenth-riders-evaluator/examples/multi-agent/logs/match/401.jsonl"


class F1TenthLogReader:

    def __init__(self, log_file):
        self.reader = jsonlines.open(log_file)
        self.keys = self.reader.read()

    def get_obs(self):
        try:
            obj = self.reader.read()
            obs = {k: np.array(obj[i]) for i, k in enumerate(self.keys)}
            obs['ego_idx'] = 0
            return obs
        except Exception:
            return None


class DummyFPSDisplay:

    def draw(self):
        return


class RidersF1TenthRenderer(EnvRenderer):

    def __init__(self, w, h):
        super(RidersF1TenthRenderer, self).__init__(w, h)
        self.fps_display = DummyFPSDisplay()
        self.paused = False

    def on_close(self):
        self.has_exit = True
        from pyglet import app
        if app.event_loop.is_running:
            self.close()

    def on_key_release(self, symbol, modifiers):
        if symbol != 32:
            return

        self.paused = not self.paused


class F1TenthVideoPlayer:
    """
    Given a log file, replays result using F1Tenth Gym Renderer
    """

    def __init__(self, log_file, map_path, w, h):
        self.reader = F1TenthLogReader(log_file)
        self.renderer = RidersF1TenthRenderer(w, h)
        self.renderer.update_map(map_path, '.png')

    def update(self, dt):
        if self.renderer.paused:
            return

        # load next observation from file,
        # when reaches end of file, reader returns None & we stop updating pyglet's renderer
        obs = self.reader.get_obs()
        if not obs:
            pyglet.clock.unschedule(self.update)
            return

        self.renderer.update_obs(obs)
        self.renderer.dispatch_events()
        self.renderer.on_draw()
        self.renderer.flip()


if __name__ == "__main__":
    player = F1TenthVideoPlayer(LOG_PATH, MAP_PATH, WINDOW_W, WINDOW_H)
    pyglet.clock.schedule_interval(player.update, 0.005)
    pyglet.app.run()

