# GitHub upload and Actions — VIO-Lite

This repository is a student visual-odometry / IMU scale-experiment project. The CI workflow runs C++ tests, Python evaluation tests and **synthetic** camera/IMU smoke data. It does **not** run EuRoC and is **not** evidence of metric scale recovery.

## Windows 11 with WSL Ubuntu

1. Extract this ZIP so the `VIO-Lite` directory is `C:\Projects\VIO-Lite`. Windows extraction sometimes creates an extra directory level: ensure `C:\Projects\VIO-Lite\CMakeLists.txt` exists.
2. Open Ubuntu using `wsl -d Ubuntu-24.04` in Windows Terminal.
3. Run:

```bash
cd /mnt/c/Projects/VIO-Lite
sudo apt-get update
sudo apt-get install -y g++ cmake libopencv-dev libeigen3-dev python3-venv python3-pip
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r scripts/requirements.txt
python scripts/run_project.py --output output-smoke
```

4. Inspect `output-smoke/` for plot and CSV outputs. The generated smoke data is synthetic, not EuRoC.
5. EuRoC (optional, separate): download official `MH_01_easy` ASL format into `datasets/MH_01_easy/mav0` as described in README, then:

```bash
python scripts/run_project.py --dataset datasets/MH_01_easy/mav0 --output output-euroc --max-frames 300
```

The scale estimate may fail on real data. Report this honestly.

## GitHub

1. Log into GitHub and click **New repository**. Name it `VIO-Lite`. Choose Public if you want recruiters to view it. Do not create a second README, license or gitignore (already included).
2. Open the new repository, choose **uploading an existing file** or **Add file → Upload files**.
3. Drag the **contents inside** the `VIO-Lite` folder, including the `.github` folder. Do not upload just the ZIP. Do not upload `.venv`, `build`, outputs or EuRoC datasets.
4. Commit the files to `main`.
5. Click **Actions**, choose the `VIO-Lite — C++ and synthetic smoke CI` workflow (if asked, enable workflows), then inspect the automatically triggered run.
6. Green check means that GitHub's runner passed its checks. Review failed steps in red runs. The workflow also supports **Run workflow**.

Read more: https://docs.github.com/en/actions/tutorials/build-and-test-code
