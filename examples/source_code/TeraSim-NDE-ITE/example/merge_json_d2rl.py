from pathlib import Path
import typer
import json


def merge_json(dir: str):
    raw_data_dir_path = Path(dir) / "raw_data" 

    d2rl_all_data_dir_path = Path(dir) / "aggregated_d2rl_data"
    d2rl_all_data_dir_path.mkdir(parents=True, exist_ok=True)
    d2rl_raw_data_dir_path = Path(dir) / "aggregated_d2rl_data" / "raw_data"
    d2rl_raw_data_dir_path.mkdir(parents=True, exist_ok=True)

    for raw_data_file in raw_data_dir_path.glob("**/monitor.json"):
        important_keys = (
            "criticality_step_info",
            "drl_obs",
            "epsilon_step_info",
            "weight_step_info",
            "finish_reason",
            "weight",
        )
        simplified_data = {}
        with open(raw_data_file, "r") as f:
            raw_data = json.load(f)
            for key in important_keys:
                simplified_data[key] = raw_data[key]
            new_file_path = d2rl_raw_data_dir_path / f"{raw_data_file.parent.name}.json"
            with open(new_file_path, "w") as f:
                json.dump(simplified_data, f, indent=4)


if __name__ == "__main__":
    experiment_path = "/home/zhijie/terasim/Driving-Intelligence-Test/output_aws/v10"
    merge_json(experiment_path)