#!/usr/bin/env python3
"""Pretrain a small NumPy MLP on synthetic 2D marker trajectories.

The saved model is used only to draw a predicted point in the camera image.
No ROS topics or robot actuator commands are produced by this training tool.
"""

import argparse
from pathlib import Path

import numpy as np


FPS = 30.0
HISTORY_LENGTH = 8
HORIZON_SECONDS = 0.25
FEATURE_SCALE = 10.0


def make_episode(rng, duration_seconds=12.0):
    """Make a smooth, bounded, varied 2D path in normalized image coordinates."""
    count = int(duration_seconds * FPS) + HISTORY_LENGTH + int(HORIZON_SECONDS * FPS) + 2
    times = np.arange(count, dtype=np.float64) / FPS
    phase = rng.uniform(0.0, 2.0 * np.pi, size=4)
    frequency = rng.uniform(0.18, 0.75, size=2)
    amplitude_x = rng.uniform(0.16, 0.34)
    amplitude_y = rng.uniform(0.14, 0.31)
    drift_x, drift_y = rng.uniform(-0.012, 0.012, size=2)

    x = (
        0.5
        + amplitude_x * np.sin(frequency[0] * times + phase[0])
        + 0.10 * np.sin(1.7 * frequency[0] * times + phase[1])
        + drift_x * (times - duration_seconds / 2.0)
    )
    y = (
        0.5
        + amplitude_y * np.sin(frequency[1] * times + phase[2])
        + 0.09 * np.sin(1.45 * frequency[1] * times + phase[3])
        + drift_y * (times - duration_seconds / 2.0)
    )
    return np.clip(np.column_stack((x, y)), 0.04, 0.96)


def make_examples(rng, episode_count):
    features = []
    labels = []
    horizon_frames = HORIZON_SECONDS * FPS
    for _ in range(episode_count):
        truth = make_episode(rng)
        last_start = len(truth) - HISTORY_LENGTH - int(np.ceil(horizon_frames)) - 1
        for start in range(0, last_start, 2):
            history = truth[start : start + HISTORY_LENGTH].copy()
            observed = history + rng.normal(0.0, 0.0025, size=history.shape)
            current = observed[-1]

            future_index = start + HISTORY_LENGTH - 1 + horizon_frames
            lower = int(np.floor(future_index))
            fraction = future_index - lower
            future = truth[lower] * (1.0 - fraction) + truth[lower + 1] * fraction

            # Express history as offsets from the latest observed point. This
            # makes the model independent of the marker's absolute image location.
            offsets = (observed - current) * FEATURE_SCALE
            features.append(offsets.reshape(-1))
            labels.append((future - current) * FEATURE_SCALE)

    return np.asarray(features, dtype=np.float32), np.asarray(labels, dtype=np.float32)


def forward(features, weights):
    hidden1 = np.tanh(features @ weights["w1"] + weights["b1"])
    hidden2 = np.tanh(hidden1 @ weights["w2"] + weights["b2"])
    output = hidden2 @ weights["w3"] + weights["b3"]
    return output


def constant_velocity_prediction(history, horizon_seconds=HORIZON_SECONDS):
    times = np.arange(HISTORY_LENGTH, dtype=np.float64) / FPS
    relative_times = times - times[-1]
    velocity = np.asarray(
        [np.polyfit(relative_times, history[:, axis], 1)[0] for axis in range(2)]
    )
    return history[-1] + velocity * horizon_seconds


def evaluate(model, episode_count, seed):
    rng = np.random.default_rng(seed)
    model_errors = []
    baseline_errors = []
    horizon_frames = HORIZON_SECONDS * FPS
    for _ in range(episode_count):
        truth = make_episode(rng)
        last_start = len(truth) - HISTORY_LENGTH - int(np.ceil(horizon_frames)) - 1
        for start in range(0, last_start, 3):
            history = truth[start : start + HISTORY_LENGTH].copy()
            observed = history + rng.normal(0.0, 0.0025, size=history.shape)
            current = observed[-1]

            future_index = start + HISTORY_LENGTH - 1 + horizon_frames
            lower = int(np.floor(future_index))
            fraction = future_index - lower
            future = truth[lower] * (1.0 - fraction) + truth[lower + 1] * fraction

            features = ((observed - current) * FEATURE_SCALE).reshape(1, -1)
            predicted_delta = forward(features, model)[0] / FEATURE_SCALE
            model_point = current + predicted_delta
            baseline_point = constant_velocity_prediction(observed)
            model_errors.append(np.linalg.norm((model_point - future) * [640.0, 480.0]))
            baseline_errors.append(np.linalg.norm((baseline_point - future) * [640.0, 480.0]))

    return float(np.mean(model_errors)), float(np.mean(baseline_errors))


def train(seed=2026, episode_count=180, epochs=200, batch_size=256):
    rng = np.random.default_rng(seed)
    train_x, train_y = make_examples(rng, episode_count)

    input_size = 2 * HISTORY_LENGTH
    hidden_size = 32
    weights = {
        "w1": rng.normal(0.0, np.sqrt(1.0 / input_size), (input_size, hidden_size)).astype(np.float32),
        "b1": np.zeros(hidden_size, dtype=np.float32),
        "w2": rng.normal(0.0, np.sqrt(1.0 / hidden_size), (hidden_size, hidden_size)).astype(np.float32),
        "b2": np.zeros(hidden_size, dtype=np.float32),
        "w3": rng.normal(0.0, np.sqrt(1.0 / hidden_size), (hidden_size, 2)).astype(np.float32),
        "b3": np.zeros(2, dtype=np.float32),
    }
    first_moment = {name: np.zeros_like(value) for name, value in weights.items()}
    second_moment = {name: np.zeros_like(value) for name, value in weights.items()}
    beta1, beta2 = 0.9, 0.999
    learning_rate = 0.002
    step = 0

    for epoch in range(epochs):
        order = rng.permutation(len(train_x))
        for batch_start in range(0, len(order), batch_size):
            indices = order[batch_start : batch_start + batch_size]
            x = train_x[indices]
            y = train_y[indices]

            z1 = x @ weights["w1"] + weights["b1"]
            h1 = np.tanh(z1)
            z2 = h1 @ weights["w2"] + weights["b2"]
            h2 = np.tanh(z2)
            prediction = h2 @ weights["w3"] + weights["b3"]

            d_output = (2.0 / len(x)) * (prediction - y)
            gradients = {
                "w3": h2.T @ d_output,
                "b3": np.sum(d_output, axis=0),
            }
            d_hidden2 = (d_output @ weights["w3"].T) * (1.0 - h2 * h2)
            gradients["w2"] = h1.T @ d_hidden2
            gradients["b2"] = np.sum(d_hidden2, axis=0)
            d_hidden1 = (d_hidden2 @ weights["w2"].T) * (1.0 - h1 * h1)
            gradients["w1"] = x.T @ d_hidden1
            gradients["b1"] = np.sum(d_hidden1, axis=0)

            step += 1
            for name, gradient in gradients.items():
                first_moment[name] = beta1 * first_moment[name] + (1.0 - beta1) * gradient
                second_moment[name] = beta2 * second_moment[name] + (1.0 - beta2) * gradient * gradient
                m_hat = first_moment[name] / (1.0 - beta1**step)
                v_hat = second_moment[name] / (1.0 - beta2**step)
                weights[name] -= learning_rate * m_hat / (np.sqrt(v_hat) + 1e-8)

        if epoch == 0 or (epoch + 1) % 20 == 0:
            loss = float(np.mean((forward(train_x, weights) - train_y) ** 2))
            print("epoch {:3d}/{:3d}  train_mse={:.6f}".format(epoch + 1, epochs, loss))

    weights["history_length"] = np.asarray(HISTORY_LENGTH, dtype=np.int32)
    weights["horizon_seconds"] = np.asarray(HORIZON_SECONDS, dtype=np.float32)
    weights["feature_scale"] = np.asarray(FEATURE_SCALE, dtype=np.float32)
    return weights


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        default=str(Path(__file__).resolve().parents[1] / "models" / "trajectory_mlp.npz"),
        help="Path for the trained NumPy model (.npz)",
    )
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--episodes", type=int, default=180)
    parser.add_argument("--epochs", type=int, default=200)
    args = parser.parse_args()

    weights = train(seed=args.seed, episode_count=args.episodes, epochs=args.epochs)
    model_error, baseline_error = evaluate(weights, episode_count=12, seed=args.seed + 1)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output_path, **weights)

    print("held-out mean error: MLP={:.2f}px, linear={:.2f}px".format(model_error, baseline_error))
    print("saved pretrained model: {}".format(output_path))


if __name__ == "__main__":
    main()
