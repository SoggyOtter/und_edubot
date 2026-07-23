import numpy as np
from line_tools.utils import line_fit, dist_2_line
import typing as t

def split_segments(segments, max_d=150, min_pts=3):
    all_valid = []
    for seg in segments:
        seg = np.asarray(seg)  # shape (2, N)
        # seg = seg.reshape(2, -1)

        # Compute distances between consecutive points
        diffs = np.diff(seg, axis=1)          # shape (2, N-1)
        dists = np.linalg.norm(diffs, axis=0) # shape (N-1,)

        # True where distance < max_d (good), False where gap is too large
        mask = dists < max_d

        # We want to split where mask == False (bad distances)
        # mask: T T T F T T  --> split after index 3
        split_idxs = np.flatnonzero(~mask) + 1

        # Split segment
        split_segs = np.split(seg, split_idxs, axis=1)

        # Keep only segments with enough points
        for s in split_segs:
            if s.shape[1] >= min_pts:
                all_valid.append(s)

    return all_valid

def iterative_fit(pts, sigma = 25):
    segments = []
    segment = np.array([pts[0]])
    # print(segment.shape)
    print(len(pts))
    for i in range(1, len(pts)):
        # print(pts.shape)
        proposed_segment = np.vstack((segment,pts[i]))
        if segment.shape[0] < 3:
            # print("FUCKS")
            segment = proposed_segment
            continue
        proposed_L = line_fit(proposed_segment[:,0], proposed_segment[:,1])
        dists = dist_2_line(proposed_segment[:,0], proposed_segment[:,1], proposed_L)
        if dists.max() > sigma:
            segments.append(segment)
            segment = np.array(pts[i])
            continue
        segment = proposed_segment
    segments.append(segment)
    return segments

def ransac_extraction(points, max_lines,n_pts = 3, max_iter = 1000, threshold = 25):
    segments = []
    current_inlier = np.array([])
    current_mask = np.array([])
    # randomly select 2-3 indices, should make this a tunable parameter
    points = points.T
    for i in range(max_lines):
        for k in range(max_iter):
            random_row_indices = np.random.choice(points.shape[0], size=n_pts, replace=False)
            # Select the random rows using fancy indexing
            random_rows = points[random_row_indices, :]
            L = line_fit(random_rows[:,0], random_rows[:,1])
            dists = np.abs(dist_2_line(points[:,0], points[:,1], L))
            inlier_mask = dists < threshold
            # print(points[inlier_mask].shape[0])
            if points[inlier_mask].shape[0] > current_inlier.shape[0]:
                current_inlier = points[inlier_mask]
                current_mask = inlier_mask


        if current_inlier.shape[0] == 0:
            break
        segments.append(current_inlier)
        points = points[~current_mask]
        current_inlier = np.array([])
        current_mask = np.array([])
        # print(points.shape[0])
        if points.shape[0] <= n_pts:
            break
    return segments

def get_first_last_points(x,y):
    return np.array([x[0],x[-1]]), np.array([y[0],y[-1]])

def endpoint_fit(x,y):
   x, y = get_first_last_points(x,y)
   rise = y[1] - y[0]
   run = x[1] - x[0]
   m = rise/run
   b = y[0] - m*x[0]
   L = np.array([m, -1 ,b])
   norm_factor = np.sqrt(L[0]**2 + L[1]**2)
   L = L/norm_factor
   return L


def iepf(x,y, sigma = 200):
    if len(x) < 2:
        return []
    try:
        L = endpoint_fit(x,y)
    except:
        return []
    dists = np.abs(dist_2_line(x,y,L))
    # print(dists.max())
    max_dist = dists.argmax()

    if dists.max() > sigma:
        # plus 1 to preserve overlap
        # To ensure that all/most points are assigned
        # if max_dist == len(x):
        #     max_dist-=1
        # elif max_dist == 0:
        #     max_dist += 1
        rec_left = iepf(x[:max_dist], y[:max_dist],sigma)
        rec_right = iepf(x[max_dist:], y[max_dist:],sigma)
        return rec_left + rec_right

    # return as segments
    return [[x,y]]


def split(x,y, thresh=50):
    # Since we want to fit a line/filter based on this data
    if len(x) == 2:
        return [[x,y]]
    try:
        L = line_fit(x, y)
    except:
        # print("failed to fit line")
        return []
    dists = np.abs(dist_2_line(x,y,L))
    # dists should all be positive so use abs
    max_dist = np.max(dists)
    max_dist_idx = np.argmax(dists)

    if max_dist < thresh:
        return [[x,y]]
    # with less than three points, line_fit returns kinda weird results
    if max_dist_idx == 0:
        max_dist_idx += 2
    elif max_dist_idx == len(x)-1:
        max_dist_idx -= 2


    return split(x[:max_dist_idx+1], y[:max_dist_idx+1]) + split(x[max_dist_idx:], y[max_dist_idx:])

def implicit_to_hough(L: t.Sequence[float]) -> t.Tuple[float, float]:
    denom = np.sqrt(L[0]**2 + L[1]**2)
    rho = - L[2] / denom
    theta = np.atan2(L[1], L[0])

    if rho < 0:
        theta += np.pi
        rho *= -1
    return rho.item(), theta.item()


def merge_segments2(Ls, segments, d_theta = np.deg2rad(35),  d_rho = 90 ):
    if not segments:
        return []

    out = [np.array(segments[0])]

    last_L = Ls[0]
    for i in range(1, len(segments)):
        rho1, theta1 = implicit_to_hough(last_L)
        rho2, theta2 = implicit_to_hough(Ls[i])
        # normalize thetas between [0, 2* pi] for easier comparison

        # theta_i = abs(theta1-theta2)
        # more stable way to get difference in angles
        theta_i = np.abs(np.arctan2(np.sin(theta1 - theta2), np.cos(theta1 - theta2)))
        rho_i = abs(rho1-rho2)

        seg_i = np.array(segments[i])

        if theta_i < d_theta and rho_i < d_rho:

            out[-1] = np.hstack((out[-1], seg_i))
            # out[-1] = split_segments(out[-1], max_d = 25)
            last_L = line_fit(out[-1][0], out[-1][1])
        else:

            out.append(seg_i)
            last_L = line_fit(seg_i[0], seg_i[1])

    if len(out) > 1:
        # Check if the first and last segments are collinear
        L_first = line_fit(out[0][0], out[0][1])
        # last_L is the line fit for out[-1] from the loop

        rho1, theta1 = implicit_to_hough(L_first)
        rho2, theta2 = implicit_to_hough(last_L)

        theta_i = np.abs(np.arctan2(np.sin(theta1 - theta2), np.cos(theta1 - theta2)))
        rho_i = abs(rho1 - rho2)

        if theta_i < d_theta and rho_i < d_rho:
            # Merge last segment into the first one
            out[0] = np.hstack((out[-1], out[0]))
            out.pop()

    return out

def merge_segments(Ls, segments, d_theta = np.deg2rad(35),  d_rho = 90 ):
    out = [segments[0]]

    last_L = Ls[0]
    for i in range(1, len(segments)):
        rho1, theta1 = implicit_to_hough(last_L)
        rho2, theta2 = implicit_to_hough(Ls[i])
        # normalize thetas between [0, 2* pi] for easier comparison

        # theta_i = abs(theta1-theta2)
        # more stable way to get difference in angles
        theta_i = np.abs(np.arctan2(np.sin(theta1 - theta2), np.cos(theta1 - theta2)))
        rho_i = abs(rho1-rho2)

        if theta_i < d_theta and rho_i < d_rho:

            out[-1] = np.hstack((np.array(out[-1]), np.array(segments[i])))
            # out[-1] = split_segments(out[-1], max_d = 25)
            last_L = line_fit(out[-1][0], out[-1][1])
        else:

            out.append(segments[i])
            last_L = line_fit(segments[i][0], segments[i][1])

        return out