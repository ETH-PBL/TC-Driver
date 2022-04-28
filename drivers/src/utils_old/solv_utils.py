# from utils.forces.MPCC_solv_10N_no_warm_no_hard_invitedguest.interface import (
#                     MPCC_solv_10N_no_warm_no_hard_invitedguest_py,
#                     )
from ..utils.forces.MPCC_solv_20N_no_warm_no_hard_invitedguest.interface import (
    MPCC_solv_20N_no_warm_no_hard_invitedguest_py,
)
from ..utils.forces.MPCC_solv_40N_no_warm_no_hard_invitedguest.interface import (
    MPCC_solv_40N_no_warm_no_hard_invitedguest_py,
)
from ..utils.forces.MPCC_solv_80N_no_warm_no_hard_invitedguest.interface import (
    MPCC_solv_80N_no_warm_no_hard_invitedguest_py,
)
from ..utils.forces.MPCC_solv_10N_no_warm_no_hard_gnone.interface import (
    MPCC_solv_10N_no_warm_no_hard_gnone_py,
)
from ..utils.forces.MPCC_solv_20N_no_warm_no_hard_gnone.interface import (
    MPCC_solv_20N_no_warm_no_hard_gnone_py,
)


class SolverTaco:
    """
    A wrapper that takes care of choosing the appropriate forces
    compiled solver
    """

    def __init__(self, cur_machine, horizon) -> None:
        self.cur_machine = cur_machine
        self.horizon = horizon

    def solve(self, params):

        if self.cur_machine == "invitedguest":
            if self.horizon == 20:
                return MPCC_solv_20N_no_warm_no_hard_invitedguest_py.MPCC_solv_20N_no_warm_no_hard_invitedguest_solve(
                    params
                )
            elif self.horizon == 40:
                return MPCC_solv_40N_no_warm_no_hard_invitedguest_py.MPCC_solv_40N_no_warm_no_hard_invitedguest_solve(
                    params
                )
            elif self.horizon == 80:
                return MPCC_solv_80N_no_warm_no_hard_invitedguest_py.MPCC_solv_80N_no_warm_no_hard_invitedguest_solve(
                    params
                )
            else:
                raise NotImplementedError(
                    "The solver with horizon {} for machine {} is either not implemented in the wrapper or not compiled at all.".format(
                        self.horizon, self.cur_machine
                    )
                )
        elif self.cur_machine == "gnone":
            if self.horizon == 10:
                return MPCC_solv_10N_no_warm_no_hard_gnone_py.MPCC_solv_10N_no_warm_no_hard_gnone_solve(
                    params
                )
            elif self.horizon == 20:
                return MPCC_solv_20N_no_warm_no_hard_gnone_py.MPCC_solv_20N_no_warm_no_hard_gnone_solve(
                    params
                )
            else:
                raise NotImplementedError(
                    "The solver with horizon {} for machine {} is either not implemented in the wrapper or not compiled at all.".format(
                        self.horizon, self.cur_machine
                    )
                )
