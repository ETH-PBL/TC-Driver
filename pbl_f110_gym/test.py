import torch
import time 

a_full = torch.randn(10240, 10240, dtype=torch.double, device='cuda')
b_full = torch.randn(10240, 10240, dtype=torch.double, device='cuda')
ab_full = a_full @ b_full
mean = ab_full.abs().mean()  # 80.7277

a = a_full.float()
b = b_full.float()

# Do matmul at TF32 mode.
torch.backends.cuda.matmul.allow_tf32 = True
time1 = time.perf_counter()
ab_tf32 = a @ b  # takes 0.016s on GA100
time2 = time.perf_counter()
print(time2-time1)
error = (ab_tf32 - ab_full).abs().max()  # 0.1747
relative_error = error / mean  # 0.0022
#print(relative_error)

# Do matmul with TF32 disabled.
torch.backends.cuda.matmul.allow_tf32 = False
time1 = time.perf_counter()
ab_fp32 = a @ b  # takes 0.11s on GA100
time2 = time.perf_counter()
print(time2-time1)
error = (ab_fp32 - ab_full).abs().max()  # 0.0031
relative_error = error / mean  # 0.000039
#print(relative_error)
