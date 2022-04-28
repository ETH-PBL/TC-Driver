import pyglet
from player.f1tenth_player import F1TenthVideoPlayer
import os

MAP_PATH = '/home/gnone/eduardo-ghignone/f1tenth_IROS2021/pkg/src/pkg/maps/SOCHI'

if __name__ == "__main__":
    sample_log = "/home/gnone/eduardo-ghignone/f1tenth_IROS2021/src/logs/Forza_PBL_p1-KU_p2-tiebreaker.jsonl"

    player = F1TenthVideoPlayer(sample_log, MAP_PATH, 1000, 800)
    pyglet.clock.schedule_interval(player.update, 0.005)
    pyglet.app.run()
