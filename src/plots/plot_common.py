import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# **ログデータを解析し、プロットを生成する関数**
def read_log_and_plot(log_filename):
    # **データの読み込み**
    df = pd.read_csv(log_filename)

    # **ヘッダをチェックし、必要なカラムを取得**
    required_columns = ["p0", "p1", "p2", "v0", "v1", "v2", "f0", "f1", "f2"]
    if not all(col in df.columns for col in required_columns):
        raise ValueError(f"CSV に必要なカラムがありません: {required_columns}")

    # **保存ディレクトリの作成**
    date_str = datetime.now().strftime("%Y-%m-%d")
    plot_dir = os.path.join("data", date_str, "plots")
    os.makedirs(plot_dir, exist_ok=True)

    # **データの取得**
    steps = df.index.to_numpy()
    positions = df[["p0", "p1", "p2"]].to_numpy()
    velocities = df[["v0", "v1", "v2"]].to_numpy()
    forces = df[["f0", "f1", "f2"]].to_numpy()

    # **3D プロット (p0, p1, p2)**
    plot_3d_trajectory(steps, positions, os.path.join(plot_dir, "3D_Trajectory.png"))

    # **2D プロット (v0, v1, v2)**
    plot_2d_data(steps, velocities, ["Velocity X", "Velocity Y", "Velocity Z"], "Velocity Over Time", os.path.join(plot_dir, "Velocity_Plot.png"))

    # **2D プロット (f0, f1, f2)**
    plot_2d_data(steps, forces, ["Force X", "Force Y", "Force Z"], "Forces Over Time", os.path.join(plot_dir, "Forces_Plot.png"))

    print(f"プロットを {plot_dir} に保存しました。")

# **3D 位置プロット**
def plot_3d_trajectory(steps, positions, save_path):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label="3D Trajectory")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()

    plt.savefig(save_path)
    plt.close()
    print(f"3D Trajectory saved to {save_path}")

# **2D プロット**
def plot_2d_data(steps, data, labels, title, save_path):
    fig, ax = plt.subplots()
    
    for i in range(data.shape[1]):
        ax.plot(steps, data[:, i], label=labels[i])
    
    ax.legend()
    ax.set_title(title)

    plt.savefig(save_path)
    plt.close()
    print(f"{title} saved to {save_path}")

# **メイン実行**
if __name__ == "__main__":
    log_filename = "data/2025-02-23/2025-02-23_simulation_log.csv"
    read_log_and_plot(log_filename)
