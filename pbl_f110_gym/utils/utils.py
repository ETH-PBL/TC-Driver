import os
from stable_baselines3.common.callbacks import CheckpointCallback

class NormalizedCheckpointCallback(CheckpointCallback):
    """
    Just a new callback that saves the statistics of the normalizer
    when the model is saved
    """

    def _on_step(self) -> bool:
        super()._on_step()

        if self.n_calls % self.save_freq == 0:
            path = os.path.join(self.save_path, f"{self.name_prefix}_{self.num_timesteps}_steps_stats.pkl")
            self.training_env.save(path)

        return True
