from typing import Optional

import numpy as np

import lib.camera as cmr
import lib.context as ctx


def get_bounding_boxes(ctx: ctx.AppContext, img: np.ndarray) -> Optional[np.ndarray]:
    results = ctx.model(img)
    bboxes = results[0].boxes.xyxy

    if bboxes.shape[0] == 0:
        return None

    bboxes = np.round(bboxes.cpu().numpy()).astype(int)

    if ctx.config.en.sanitize:
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

        pt1 = np.array(
            [(org_bbox[0] + org_bbox[2]) // 2, (org_bbox[1] + org_bbox[3]) // 2]
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

            pt2 = np.array(
                [(comp_bbox[0] + comp_bbox[2]) // 2, (comp_bbox[1] + comp_bbox[3]) // 2]
            )

            dist = np.linalg.norm(pt1 - pt2)

            if dist < ctx.config.od.d_cells:
                keep[i] = False
                keep[j] = False

    return bboxes[keep]


def object_detection(ctx: ctx.AppContext, logger):
    logger.info("Capturing original image...")
    try:
        od_img = cmr.return_image(ctx.camera)
    except Exception as e:
        logger.error(f"Error capturing image: {e}")
        raise e

    logger.info("Detecting objects...")
    bboxes = get_bounding_boxes(ctx, od_img)

    if bboxes is None or bboxes.shape[0] == 0:
        logger.warning(
            "There are no valid objects detected. Skipping current position..."
        )
        raise ValueError("No objects detected.")

    logger.debug(f"Detected {len(bboxes)} objects.")

    return bboxes
