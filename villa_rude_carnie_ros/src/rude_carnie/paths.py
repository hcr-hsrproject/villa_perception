import os


def get_age_checkpoint_path():
    script_path = os.path.realpath(__file__)
    package_directory = os.path.abspath(os.path.join(script_path, os.pardir, os.pardir, os.pardir))
    return os.path.join(package_directory,  "checkpoints", "age_identification_checkpoint")

def get_gender_checkpoint_path():
    script_path = os.path.realpath(__file__)
    package_directory = os.path.abspath(os.path.join(script_path, os.pardir, os.pardir, os.pardir))
    return os.path.join(package_directory, "checkpoints", "gender_identification_checkpoint")