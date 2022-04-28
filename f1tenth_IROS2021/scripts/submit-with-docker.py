import docker
import os
from getpass import getpass

SCRIPTS_DIR = os.path.abspath(os.path.dirname(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPTS_DIR)
SUBMIT_DOCKERFILE_PATH = os.path.join(PROJECT_ROOT, 'compose', 'submit', 'Dockerfile')
AGENT_DOCKERFILE_PATH = os.path.join(PROJECT_ROOT, 'compose', 'agent', 'Dockerfile')
AGENT_PARENT = "registry.gitlab.com/acrome-colab/riders-poc/f1tenth-riders-quickstart/roscore:latest"


def build_agent_image():
    client = docker.from_env()
    client.images.build(
        path=PROJECT_ROOT,
        dockerfile=AGENT_DOCKERFILE_PATH,
        tag="a1",
        buildargs={
            "PARENT": AGENT_PARENT
        }
    )


def build_submit_image():
    client = docker.from_env()
    client.images.build(
        path=PROJECT_ROOT,
        dockerfile=SUBMIT_DOCKERFILE_PATH,
        tag="registry.gitlab.com/acrome-colab/riders-poc/f1tenth-riders-quickstart/submit",
    )


def run_submit_image(envs):
    client = docker.from_env()
    client.containers.run(
        "registry.gitlab.com/acrome-colab/riders-poc/f1tenth-riders-quickstart/submit",
        environment=envs
    )


if __name__ == '__main__':
    print("Building your Agent Docker image, this may take a while...")
    build_agent_image()

    print("Image built & tagged successfully.")
    print("--------------------------------------")
    print("This script will compile your project & upload to Riders.ai for evaluation.")
    print("Please enter your Riders.ai Username & Password")

    username = input("Username: ")
    password = getpass("Password: ")

    print("--------------------------------------")
    print("Please enter a short description of your submission. This would help you compare separate submissions")
    description = input("Description: ")

    print("--------------------------------------")
    print("Building submit image...")
    build_submit_image()

    print("Starting submission in Docker")
    run_submit_image({
        "RIDERS_USERNAME": username,
        "RIDERS_PASSWORD": password,
        "DESCRIPTION": description,
    })

    print("Submission successful! Please go to https://riders.ai/challenge/40/submissions to validate.")
    print("Please also visit https://riders.ai/challenge/40/result page"
          " in 10-15 minutes to view results of your agent.")
