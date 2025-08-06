import numpy as np

from lib.camera import return_range
import lib.context as ctx
from pipython import pitools
from scipy.optimize import golden

RAD = 40


def measure_std_dev(img):
    f = np.fft.fftshift(np.fft.fft2(img))

    height, width = f.shape
    cy, cx = height // 2, width // 2
    y, x = np.ogrid[:height, :width]
    dist_sq = (y - cy) ** 2 + (x - cx) ** 2

    # signal = np.abs(f[cy, cx:])
    # smoothed_signal = gaussian_filter1d(signal, sigma=2.0)
    # derivative = np.gradient(smoothed_signal)
    # rad = find_valley_index(derivative, 0) + 10

    f[(dist_sq > RAD**2)] = 0

    F = np.fft.ifft2(np.fft.ifftshift(f))
    std_dev = F.std()
    return std_dev


def get_avg(pos, ctx: ctx.AppContext, func):
    l_pos = max(
        ctx.config.focus.z_min,
        pos - ctx.config.focus.step_num * ctx.config.focus.step_finer,
    )

    r_pos = min(
        ctx.config.focus.z_max,
        pos + ctx.config.focus.step_num * ctx.config.focus.step_finer,
    )

    imgs = return_range(ctx, l_pos, r_pos)

    scores = []
    for img in imgs:
        score = func(img)
        scores.append(score)

    return np.mean(scores) if len(scores) > 0 else np.inf


def autofocus_golden(ctx: ctx.AppContext, score_func, tol=None):
    step = ctx.config.focus.step_finer
    z_min = ctx.config.focus.z_min
    z_max = ctx.config.focus.z_max

    cache = {}

    def wrapped_func(pos):
        quant_pos = round(pos / step) * step

        if quant_pos in cache:
            return cache[quant_pos]

        val = get_avg(quant_pos, ctx, score_func)
        cache[quant_pos] = val
        return val

    tol = tol if tol is not None else step

    best_pos = golden(wrapped_func, brack=(z_min, z_max), tol=tol)
    best_pos = round(best_pos / step) * step

    ctx.pidevice.MOV(ctx.config.axes.z, best_pos)
    pitools.waitontarget(ctx.pidevice, ctx.config.axes.z)


def autofocus_hill_climbing(ctx: ctx.AppContext, func):
    focus_cache = {}

    def cached_get_avg(pos):
        if pos not in focus_cache:
            focus_cache[pos] = get_avg(pos, ctx, func)
        return focus_cache[pos]

    starting_pos = ctx.pidevice.qPOS(ctx.config.axes.z)[ctx.config.axes.z]
    direction = 1
    step = ctx.config.focus.step_coarse

    pos_check = starting_pos + direction * step
    if pos_check >= ctx.config.focus.z_max or pos_check < ctx.config.focus.z_min:
        return starting_pos

    if cached_get_avg(pos_check) > cached_get_avg(starting_pos):
        direction *= -1
        current_pos = starting_pos + 2 * direction * step
    else:
        current_pos = starting_pos + direction * step

    current_pos = max(
        ctx.config.focus.z_min, min(current_pos, ctx.config.focus.z_max - 1)
    )

    while True:
        next_pos = current_pos + direction * step
        if (
            next_pos >= ctx.config.focus.z_max
            or next_pos < ctx.config.focus.z_min
            or cached_get_avg(next_pos) > cached_get_avg(current_pos)
        ):
            direction *= -1
            break
        current_pos = next_pos

    step = ctx.config.focus.step_fine
    while True:
        next_pos = current_pos + direction * step
        if (
            next_pos >= ctx.config.focus.z_max
            or next_pos < ctx.config.focus.z_min
            or cached_get_avg(next_pos) > cached_get_avg(current_pos)
        ):
            break
        current_pos = next_pos
    ctx.pidevice.MOV(ctx.config.axes.z, current_pos)
    pitools.waitontarget(ctx.pidevice, ctx.config.axes.z)

    # Might return back to starting idx
    # ctx.pidevice.MOV(ctx.config.axes.z, starting_pos)
    # pitools.waitontarget(ctx.pidevice, ctx.config.axes.z)
