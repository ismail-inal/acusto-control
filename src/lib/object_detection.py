from typing import Optional

import numpy as np

import lib.context as ctx


def get_bounding_boxes(ctx: ctx.AppContext, img_path: str) -> Optional[np.ndarray]:
    results = ctx.model(img_path)
    bboxes = results[0].boxes.xyxy

    if bboxes.shape[0] == 0:
        return None

    bboxes = np.round(bboxes.cpu().numpy()).astype(int)
    bboxes = sanitize_mask(bboxes, ctx)

    bboxes[:, 0] -= ctx.config.od.buffer_size
    bboxes[:, 1] -= ctx.config.od.buffer_size
    bboxes[:, 2] += ctx.config.od.buffer_size
    bboxes[:, 3] += ctx.config.od.buffer_size

    return bboxes


def sanitize_mask(bboxes, ctx: ctx.AppContext):
    n = len(bboxes)
    keep = np.ones(n, dtype=bool)

    for i in range(n):
        if not keep[i]:
            continue

        org_bbox = bboxes[i]

        if (
            org_bbox[0] < ctx.config.od.d_boundary
            or org_bbox[2] > ctx.width_max - ctx.config.od.d_boundary
        ):
            keep[i] = False
            continue

        if (
            org_bbox[1] < ctx.config.od.d_boundary
            or org_bbox[3] > ctx.height_max - ctx.config.od.d_boundary
        ):
            keep[i] = False
            continue

        cx_1, cy_1 = (
            (org_bbox[0] + org_bbox[2]) // 2,
            (org_bbox[1] + org_bbox[3]) // 2,
        )

        for j in range(i + 1, n):
            if not keep[j]:
                continue

            comp_bbox = bboxes[j]
            if (
                comp_bbox[0] < ctx.config.od.d_boundary
                or comp_bbox[2] > ctx.width_max - ctx.config.od.d_boundary
            ):
                keep[j] = False
                continue

            if (
                comp_bbox[1] < ctx.config.od.d_boundary
                or comp_bbox[3] > ctx.height_max - ctx.config.od.d_boundary
            ):
                keep[j] = False
                continue

            cx_2, cy_2 = (
                (comp_bbox[0] + comp_bbox[2]) // 2,
                (comp_bbox[1] + comp_bbox[3]) // 2,
            )
            dist = np.sqrt((cx_1 - cx_2) ** 2 + (cy_1 - cy_2) ** 2)

            if dist < ctx.config.od.d_cells:
                keep[i] = False

    return bboxes[keep]
