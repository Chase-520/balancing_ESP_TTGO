/**
 * @file    scara_ik.h
 * @brief   Inverse & Forward Kinematics for a 5-bar Dual-Arm SCARA Mechanism
 *
 * @details Solves the joint angles (theta1, theta2) of a symmetric 5-bar
 *          parallel SCARA robot given a desired end-effector position (x, y).
 *
 *          Mechanism layout (top view):
 *
 *              O1 ──── upper_arm ──── E1 ──── lower_arm ────┐
 *                                                            P  (end-effector)
 *              O2 ──── upper_arm ──── E2 ──── lower_arm ────┘
 *
 *          O1, O2  : fixed base (driver) joints
 *          E1, E2  : passive elbow joints
 *          P       : end-effector (tool centre point)
 *
 *          Default geometry (editable via constructor or macros):
 *            - Base distance   d  = 7.5  cm
 *            - Upper arm       l1 = 5.0  cm
 *            - Lower arm       l2 = 12.5 cm
 *
 *          Coordinate system:
 *            - Origin at midpoint of O1–O2
 *            - X axis along O1→O2  (O1 at x = -d/2, O2 at x = +d/2)
 *            - Y axis pointing away from the base (forward)
 *            - Angles in radians, measured CCW from +X
 *
 * @section usage Usage
 * @code
 *   #include "scara_ik.h"
 *
 *   ScaraIK ik;                        // uses default geometry
 *   ScaraIKResult result;
 *
 *   if (ik.solve(0.0f, 8.0f, result)) {
 *       Serial.print("theta1 = "); Serial.println(result.theta1);
 *       Serial.print("theta2 = "); Serial.println(result.theta2);
 *   } else {
 *       Serial.println("Target unreachable");
 *   }
 * @endcode
 *
 * @author  (your name)
 * @date    2026
 * @version 1.0.0
 * @license MIT
 *
 * @note    Header-only — no .cpp needed. Requires only <math.h>.
 *          Compatible with Arduino / PlatformIO (AVR, ESP32, STM32, RP2040, …).
 */

#pragma once

#include <math.h>   /* sqrtf, atan2f, acosf, fabsf */

/* ─────────────────────────────────────────────────────────────────────────────
 * Default geometry macros (override before #include if needed)
 * ───────────────────────────────────────────────────────────────────────────*/

/** Distance between the two fixed base joints O1 and O2, in centimetres. */
#ifndef SCARA_BASE_DISTANCE
#define SCARA_BASE_DISTANCE  7.5f
#endif

/** Proximal (upper) arm length, in centimetres. Both arms are equal. */
#ifndef SCARA_UPPER_ARM
#define SCARA_UPPER_ARM      5.0f
#endif

/** Distal (lower) arm length, in centimetres. Both arms are equal. */
#ifndef SCARA_LOWER_ARM
#define SCARA_LOWER_ARM      12.5f
#endif

/** Numerical tolerance used for reachability checks. */
#ifndef SCARA_EPSILON
#define SCARA_EPSILON        1e-6f
#endif


/* ─────────────────────────────────────────────────────────────────────────────
 * Elbow configuration flag
 * ───────────────────────────────────────────────────────────────────────────*/

/**
 * @brief Selects which elbow configuration the IK solver returns.
 *
 * ELBOW_UP   (+1) keeps elbows above the base line (normal operating posture).
 * ELBOW_DOWN (-1) keeps elbows below the base line.
 */
enum ScaraElbow : int8_t {
    ELBOW_UP   =  1,
    ELBOW_DOWN = -1
};


/* ─────────────────────────────────────────────────────────────────────────────
 * Result structs
 * ───────────────────────────────────────────────────────────────────────────*/

/**
 * @brief Output of a successful inverse-kinematics solve.
 *
 * Both angles are in radians, measured counter-clockwise from the +X axis.
 * theta1 belongs to the LEFT  driver (O1, at x = -d/2).
 * theta2 belongs to the RIGHT driver (O2, at x = +d/2).
 */
struct ScaraIKResult {
    float theta1;   /**< Left  driver joint angle [rad] */
    float theta2;   /**< Right driver joint angle [rad] */
};

/**
 * @brief Output of a forward-kinematics evaluation.
 */
struct ScaraFKResult {
    float e1x;  /**< Left  elbow X  [cm] */
    float e1y;  /**< Left  elbow Y  [cm] */
    float e2x;  /**< Right elbow X  [cm] */
    float e2y;  /**< Right elbow Y  [cm] */
    float px;   /**< End-effector X [cm] */
    float py;   /**< End-effector Y [cm] */
};


/* ─────────────────────────────────────────────────────────────────────────────
 * ScaraIK — main class
 * ───────────────────────────────────────────────────────────────────────────*/

/**
 * @brief Kinematics solver for a symmetric 5-bar SCARA mechanism.
 *
 * All lengths are in the same unit (default: centimetres). Angles are always
 * in radians.  The class is stateless — every call to solve() or forwardK()
 * is independent; no internal state is mutated.
 */
class ScaraIK {
public:

    /* ── Construction ──────────────────────────────────────────────────── */

    /**
     * @brief Default constructor — uses the macro-defined geometry.
     */
    ScaraIK()
        : _d (SCARA_BASE_DISTANCE)
        , _l1(SCARA_UPPER_ARM)
        , _l2(SCARA_LOWER_ARM)
    {}

    /**
     * @brief Custom-geometry constructor.
     *
     * @param baseDistance  Distance between O1 and O2 [same unit as arms]
     * @param upperArm      Proximal link length (l1)
     * @param lowerArm      Distal  link length (l2)
     */
    ScaraIK(float baseDistance, float upperArm, float lowerArm)
        : _d (baseDistance)
        , _l1(upperArm)
        , _l2(lowerArm)
    {}


    /* ── Inverse Kinematics ─────────────────────────────────────────────── */

    /**
     * @brief Compute joint angles for a desired end-effector position.
     *
     * @details Uses the law of cosines applied independently to each 2-DOF
     *          serial sub-chain (O1→E1→P and O2→E2→P), then combines the
     *          two shoulder angles into a single 5-bar solution.
     *
     *          The left arm uses +elbowSign for its elbow bend direction;
     *          the right arm uses -elbowSign, which produces the symmetric
     *          posture that keeps the mechanism geometrically valid.
     *
     * @param[in]  px        Desired end-effector X position [cm]
     * @param[in]  py        Desired end-effector Y position [cm]
     * @param[out] result    Solved joint angles (only valid when return == true)
     * @param[in]  elbow     Elbow configuration (default: ELBOW_UP)
     *
     * @return true  if the target is reachable and result is populated
     * @return false if the target is outside the reachable workspace
     */
    bool solve(float px, float py,
               ScaraIKResult &result,
               ScaraElbow elbow = ELBOW_UP) const
    {
        /* Base joint positions (centred at origin) */
        const float o1x = -_d * 0.5f;
        const float o2x =  _d * 0.5f;
        const float oy  =  0.0f;

        /* Solve each arm independently */
        float theta1, theta2;

        if (!_solveSingleArm(o1x, oy, px, py,  (int8_t)elbow, theta1))
            return false;

        if (!_solveSingleArm(o2x, oy, px, py, -(int8_t)elbow, theta2))
            return false;

        result.theta1 = theta1;
        result.theta2 = theta2;
        return true;
    }


    /* ── Forward Kinematics ─────────────────────────────────────────────── I don't really need this part*/

    /**
     * @brief Compute end-effector position and elbow positions from joint angles.
     *
     * @details
     *  1. Elbow positions are found by stepping along each upper arm:
     *       E1 = O1 + l1 * [cos(theta1), sin(theta1)]
     *       E2 = O2 + l1 * [cos(theta2), sin(theta2)]
     *  2. End-effector P is the intersection of two circles of radius l2
     *     centred at E1 and E2.  The intersection above the base line is
     *     selected (consistent with ELBOW_UP convention).
     *
     * @param[in]  theta1   Left  driver joint angle [rad]
     * @param[in]  theta2   Right driver joint angle [rad]
     * @param[out] result   Populated with elbow and end-effector positions
     *
     * @return true  if a valid end-effector position exists
     * @return false if the elbow positions are incompatible (degenerate config)
     */
    bool forwardK(float theta1, float theta2,
                  ScaraFKResult &result) const
    {
        const float o1x = -_d * 0.5f;
        const float o2x =  _d * 0.5f;
        const float oy  =  0.0f;

        /* Elbow positions via upper-arm FK */
        result.e1x = o1x + _l1 * cosf(theta1);
        result.e1y = oy  + _l1 * sinf(theta1);

        result.e2x = o2x + _l1 * cosf(theta2);
        result.e2y = oy  + _l1 * sinf(theta2);

        /* End-effector: upper circle–circle intersection of two l2 circles */
        return _circleIntersect(result.e1x, result.e1y,
                                result.e2x, result.e2y,
                                _l2,
                                result.px,  result.py);
    }


    /* ── Workspace query ────────────────────────────────────────────────── */

    /**
     * @brief Quick reachability test — no angle computation.
     *
     * @param px  End-effector X [cm]
     * @param py  End-effector Y [cm]
     * @return true if (px, py) lies inside the reachable workspace of BOTH arms.
     */
    bool isReachable(float px, float py) const
    {
        ScaraIKResult dummy;
        return solve(px, py, dummy);
    }

    /**
     * @brief Maximum reach of one arm from its base joint (= l1 + l2).
     */
    float maxReach() const { return _l1 + _l2; }

    /**
     * @brief Minimum reach of one arm from its base joint (= |l2 - l1|).
     */
    float minReach() const { return fabsf(_l2 - _l1); }


    /* ── Geometry accessors ─────────────────────────────────────────────── */

    float baseDistance() const { return _d;  }  /**< O1–O2 separation [cm] */
    float upperArm()     const { return _l1; }  /**< Proximal link length  */
    float lowerArm()     const { return _l2; }  /**< Distal  link length   */


/* ─────────────────────────────────────────────────────────────────────────────
 * Private implementation
 * ───────────────────────────────────────────────────────────────────────────*/
private:

    float _d;   /**< Base joint separation  */
    float _l1;  /**< Upper (proximal) arm   */
    float _l2;  /**< Lower (distal)   arm   */

    /**
     * @brief  2-DOF serial arm IK — shoulder angle for one sub-chain.
     *
     * @details Given a fixed joint at (ox, oy) and a target at (px, py),
     *          applies the law of cosines to find the shoulder angle theta
     *          such that a two-link arm (l1, l2) reaches the target.
     *
     *          Derivation:
     *            dist  = ||(px,py) - (ox,oy)||
     *            phi   = atan2(py-oy, px-ox)          // bearing to target
     *            alpha = acos((dist² + l1² - l2²)     // law of cosines
     *                         / (2·dist·l1))
     *            theta = phi + elbowSign * alpha
     *
     * @param[in]  ox, oy     Fixed base joint position
     * @param[in]  px, py     Target position
     * @param[in]  elbowSign  +1 or -1 (selects one of two IK branches)
     * @param[out] theta      Shoulder angle [rad] (valid only when true)
     *
     * @return true if the target is within reach of this single arm
     */
    bool _solveSingleArm(float ox, float oy,
                         float px, float py,
                         int8_t elbowSign,
                         float &theta) const
    {
        const float dx    = px - ox;
        const float dy    = py - oy;
        const float dist2 = dx * dx + dy * dy;
        const float dist  = sqrtf(dist2);

        /* ── Reachability check ──────────────────────────────────────────
         * The target must lie in the annulus [|l2-l1|, l1+l2] around the
         * base joint.  A small epsilon guards against floating-point edge
         * cases exactly on the boundary.
         */
        const float reach_min = fabsf(_l1 - _l2) + SCARA_EPSILON;
        const float reach_max = _l1 + _l2         - SCARA_EPSILON;

        if (dist < reach_min || dist > reach_max) return false;

        float cosAlpha = (dist2 + _l1 * _l1 - _l2 * _l2)
                         / (2.0f * dist * _l1);

        /* Clamp to [-1, 1] to absorb floating-point overshoot */
        if (cosAlpha >  1.0f) cosAlpha =  1.0f;
        if (cosAlpha < -1.0f) cosAlpha = -1.0f;

        const float alpha = acosf(cosAlpha);

        /* ── Bearing from base joint to target ───────────────────────────*/
        const float phi = atan2f(dy, dx);

        /* ── Select elbow branch ─────────────────────────────────────────*/
        theta = phi + (float)elbowSign * alpha;
        return true;
    }

    /**
     * @brief  Find the "upper" intersection of two circles of equal radius r.
     *
     * @details Both circles are centred at (ax, ay) and (bx, by) with
     *          radius r.  Of the two intersection points, the one with the
     *          larger Y value is returned (consistent with ELBOW_UP).
     *
     *          Algorithm:
     *            d    = ||(bx,by) - (ax,ay)||
     *            a    = d / 2                       // distance to mid-point
     *            h    = sqrt(r² - a²)               // half-chord height
     *            mid  = (A + B) / 2
     *            perp = normalised perpendicular to AB
     *            P    = mid ± h * perp
     *
     * @param[in]  ax, ay   Centre of first  circle
     * @param[in]  bx, by   Centre of second circle
     * @param[in]  r        Radius (same for both)
     * @param[out] px, py   Intersection point (upper)
     *
     * @return true if circles intersect (d > 0 and d ≤ 2r)
     */
    bool _circleIntersect(float ax, float ay,
                          float bx, float by,
                          float r,
                          float &px, float &py) const
    {
        const float dx = bx - ax;
        const float dy = by - ay;
        const float d  = sqrtf(dx * dx + dy * dy);

        if (d < SCARA_EPSILON || d > 2.0f * r - SCARA_EPSILON) return false;

        const float a   = d * 0.5f;
        const float h   = sqrtf(r * r - a * a);

        /* Mid-point of the chord */
        const float mx  = (ax + bx) * 0.5f;
        const float my  = (ay + by) * 0.5f;

        /* Unit perpendicular to AB */
        const float perpX = -dy / d;
        const float perpY =  dx / d;

        /* Two candidate intersection points */
        const float p1x = mx + h * perpX;
        const float p1y = my + h * perpY;
        const float p2x = mx - h * perpX;
        const float p2y = my - h * perpY;

        /* Choose the point with the greater Y (above base line) */
        if (p1y >= p2y) { px = p1x; py = p1y; }
        else             { px = p2x; py = p2y; }

        return true;
    }
};