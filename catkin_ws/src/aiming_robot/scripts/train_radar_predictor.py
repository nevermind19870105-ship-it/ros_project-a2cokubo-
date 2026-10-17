#!/usr/bin/env python3
"""Pretrain an MLP to forecast FMCW radar target tracks in world XY coordinates."""

import argparse
from pathlib import Path

import numpy as np


HISTORY_LENGTH = 8
SAMPLE_INTERVAL = 0.25
HORIZON_SECONDS = 0.25
FEATURE_SCALE = 2.0
MEASUREMENT_NOISE = 0.035


def make_episode(rng, duration=18.0):
    """Generate a smooth but non-constant-velocity 2D target path in meters."""
    count = int(duration / SAMPLE_INTERVAL) + HISTORY_LENGTH + 4
    times = np.arange(count, dtype=np.float64) * SAMPLE_INTERVAL
    phase = rng.uniform(0.0, 2.0 * np.pi, size=6)
    wx, wy = rng.uniform(0.12, 0.72, size=2)
    ax, ay = rng.uniform(0.35, 1.15, size=2)
    drift_x, drift_y = rng.uniform(-0.12, 0.12, size=2)
    # Independent harmonics and slow changing components produce curves and
    # turns that a constant-velocity extrapolator cannot represent well.
    x = (3.0 + ax * np.sin(wx * times + phase[0])
         + 0.28 * np.sin(2.2 * wx * times + phase[1])
         + 0.18 * np.sin(0.16 * times + phase[2]) + drift_x * (times - duration / 2.0))
    y = (0.2 + ay * np.sin(wy * times + phase[3])
         + 0.24 * np.sin(2.0 * wy * times + phase[4])
         + 0.16 * np.sin(0.19 * times + phase[5]) + drift_y * (times - duration / 2.0))
    return np.column_stack((np.clip(x, 0.8, 7.5), np.clip(y, -3.5, 3.5)))


def make_examples(rng, episode_count):
    inputs, labels = [], []
    horizon_steps = int(round(HORIZON_SECONDS / SAMPLE_INTERVAL))
    for _ in range(episode_count):
        path = make_episode(rng)
        for start in range(len(path) - HISTORY_LENGTH - horizon_steps):
            truth_history = path[start : start + HISTORY_LENGTH]
            observed = truth_history + rng.normal(0.0, MEASUREMENT_NOISE, truth_history.shape)
            current = observed[-1]
            future = path[start + HISTORY_LENGTH - 1 + horizon_steps]
            inputs.append(((observed - current) / FEATURE_SCALE).reshape(-1))
            labels.append((future - current) / FEATURE_SCALE)
    return np.asarray(inputs, dtype=np.float32), np.asarray(labels, dtype=np.float32)


def forward(features, weights):
    h1 = np.tanh(features @ weights["w1"] + weights["b1"])
    h2 = np.tanh(h1 @ weights["w2"] + weights["b2"])
    return h2 @ weights["w3"] + weights["b3"]


def linear_prediction(history):
    times = np.arange(len(history), dtype=np.float64) * SAMPLE_INTERVAL
    times -= times[-1]
    design = np.column_stack((times, np.ones_like(times)))
    result = []
    for axis in range(2):
        velocity = np.linalg.lstsq(design, history[:, axis], rcond=None)[0][0]
        result.append(history[-1, axis] + velocity * HORIZON_SECONDS)
    return np.asarray(result)


def evaluate(weights, episode_count, seed):
    rng = np.random.default_rng(seed)
    mlp_errors, linear_errors = [], []
    future_step = int(round(HORIZON_SECONDS / SAMPLE_INTERVAL))
    for _ in range(episode_count):
        truth = make_episode(rng)
        for start in range(0, len(truth) - HISTORY_LENGTH - future_step, 3):
            observed = truth[start : start + HISTORY_LENGTH] + rng.normal(
                0.0, MEASUREMENT_NOISE, (HISTORY_LENGTH, 2)
            )
            current = observed[-1]
            future = truth[start + HISTORY_LENGTH - 1 + future_step]
            features = ((observed - current) / FEATURE_SCALE).reshape(1, -1)
            mlp = current + forward(features, weights)[0] * FEATURE_SCALE
            linear = linear_prediction(observed)
            mlp_errors.append(np.linalg.norm(mlp - future))
            linear_errors.append(np.linalg.norm(linear - future))
    return float(np.mean(mlp_errors)), float(np.mean(linear_errors))


def train(seed=2026, episode_count=240, epochs=180, batch_size=256):
    rng = np.random.default_rng(seed)
    train_x, train_y = make_examples(rng, episode_count)
    hidden = 32
    weights = {
        "w1": rng.normal(0, np.sqrt(1.0 / (2 * HISTORY_LENGTH)), (2 * HISTORY_LENGTH, hidden)).astype(np.float32),
        "b1": np.zeros(hidden, dtype=np.float32),
        "w2": rng.normal(0, np.sqrt(1.0 / hidden), (hidden, hidden)).astype(np.float32),
        "b2": np.zeros(hidden, dtype=np.float32),
        "w3": rng.normal(0, np.sqrt(1.0 / hidden), (hidden, 2)).astype(np.float32),
        "b3": np.zeros(2, dtype=np.float32),
    }
    m = {name: np.zeros_like(value) for name, value in weights.items()}
    v = {name: np.zeros_like(value) for name, value in weights.items()}
    step, beta1, beta2, learning_rate = 0, 0.9, 0.999, 0.0015
    for epoch in range(epochs):
        order = rng.permutation(len(train_x))
        for offset in range(0, len(order), batch_size):
            idx = order[offset : offset + batch_size]
            x, y = train_x[idx], train_y[idx]
            h1 = np.tanh(x @ weights["w1"] + weights["b1"])
            h2 = np.tanh(h1 @ weights["w2"] + weights["b2"])
            output = h2 @ weights["w3"] + weights["b3"]
            d3 = (2.0 / len(x)) * (output - y)
            grads = {"w3": h2.T @ d3, "b3": np.sum(d3, axis=0)}
            d2 = (d3 @ weights["w3"].T) * (1.0 - h2 * h2)
            grads["w2"], grads["b2"] = h1.T @ d2, np.sum(d2, axis=0)
            d1 = (d2 @ weights["w2"].T) * (1.0 - h1 * h1)
            grads["w1"], grads["b1"] = x.T @ d1, np.sum(d1, axis=0)
            step += 1
            for name, grad in grads.items():
                m[name] = beta1 * m[name] + (1.0 - beta1) * grad
                v[name] = beta2 * v[name] + (1.0 - beta2) * grad * grad
                m_hat = m[name] / (1.0 - beta1**step)
                v_hat = v[name] / (1.0 - beta2**step)
                weights[name] -= learning_rate * m_hat / (np.sqrt(v_hat) + 1e-8)
        if epoch == 0 or (epoch + 1) % 30 == 0:
            loss = float(np.mean((forward(train_x, weights) - train_y) ** 2))
            print("epoch {:3d}/{:3d} train_mse={:.6f}".format(epoch + 1, epochs, loss))

    weights["history_length"] = np.asarray(HISTORY_LENGTH, dtype=np.int32)
    weights["sample_interval"] = np.asarray(SAMPLE_INTERVAL, dtype=np.float32)
    weights["horizon_seconds"] = np.asarray(HORIZON_SECONDS, dtype=np.float32)
    weights["feature_scale"] = np.asarray(FEATURE_SCALE, dtype=np.float32)
    return weights


def main():
    default_output = Path(__file__).resolve().parents[1] / "models" / "radar_trajectory_mlp.npz"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=str(default_output))
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--episodes", type=int, default=240)
    parser.add_argument("--epochs", type=int, default=180)
    args = parser.parse_args()
    model = train(seed=args.seed, episode_count=args.episodes, epochs=args.epochs)
    mlp_error, linear_error = evaluate(model, episode_count=24, seed=args.seed + 1)
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, **model)
    print("held-out mean error: radar MLP={:.3f} m, linear={:.3f} m".format(mlp_error, linear_error))
    print("saved radar model: {}".format(path))


if __name__ == "__main__":
    main()
