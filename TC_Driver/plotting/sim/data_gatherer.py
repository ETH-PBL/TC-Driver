import yaml
import numpy as np
import pandas as pd

class DataGatherer:
    def __init__(self, max_eplen: int, n_exp: int) -> None:
        self.n_exp = n_exp

        self.data = dict()
        self.data['s_x'] = np.zeros((max_eplen, n_exp))
        self.data['s_y'] = np.zeros((max_eplen, n_exp))
        self.data['velocity'] = np.zeros((max_eplen, n_exp))
        self.data['dist_traj'] = np.zeros((max_eplen, n_exp))
        self.data['yaw'] = np.zeros((max_eplen, n_exp))

    def save_data(self, filepath: str) -> None:
        """
        saves the results of the experiment in the specified folder
        """
        for key in self.data.keys():
            # idx names
            names = [f"{key}_{i}" for i in range(self.n_exp)]

            # pandify
            df = pd.DataFrame(self.data[key], columns=names)

            # save
            df.to_csv(f"{filepath}/{key}.csv")

        