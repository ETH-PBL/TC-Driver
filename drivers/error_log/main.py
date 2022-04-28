
import yaml

with open("./sol_dump.yaml", "r") as dump:
    solution = yaml.load(dump, yaml.Loader)

with open("./param_dump.yaml", "r") as dump:
    params = yaml.load(dump, yaml.Loader)


print(params)