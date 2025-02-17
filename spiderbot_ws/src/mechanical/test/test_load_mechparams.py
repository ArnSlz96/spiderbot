import pytest
from mechanical import load_mechanical_params

def test_correct_param_loading():
    params = load_mechanical_params.load_mechanical_params()
    params_file_name = params["NAME"]

    assert params_file_name == "SPIDERBOT", "Could not read mechanical params yaml file."