#ifndef MESHBSPHEADER
#define MESHBSPHEADER

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "../utils/Scene/sceneSchema.hpp"

/**
 * Represents a single triangle in the mesh BSP tree.
 * Stores transformed world-space vertices, the face normal,
 * and a pointer to the parent ObjectData for material access.
 */
struct MeshTriangle {
    Ponto v0, v1, v2;
    Vetor normal;
    ObjectData* object;

    MeshTriangle()
        : v0(0, 0, 0),
          v1(0, 0, 0),
          v2(0, 0, 0),
          normal(0, 0, 0),
          object(nullptr) {}
};

/**
 * BSP Tree Node.
 *
 * Each internal node stores a splitting plane (from a triangle)
 * and all triangles that lie exactly on that plane (the splitter
 * plus any coplanar triangles).  Straddling triangles are split
 * into front/back fragments stored in the respective child subtrees.
 *
 * Leaf nodes contain a small set of triangles (≤ leafThreshold).
 */
struct MeshBSPNode {
    bool isLeaf;
    Vetor splitNormal;                    // splitting plane normal (internal)
    double splitD;                        // plane constant (internal)
    std::vector<MeshTriangle> triangles;  // triangles stored at this node
    std::unique_ptr<MeshBSPNode> front;   // positive side
    std::unique_ptr<MeshBSPNode> back;    // negative side

    MeshBSPNode()
        : isLeaf(true),
          splitNormal(0, 0, 0),
          splitD(0.0),
          front(nullptr),
          back(nullptr) {}
};

/** Result of classifying a triangle against a splitting plane. */
enum class PlaneSide {
    FRONT,     // all vertices on the positive side
    BACK,      // all vertices on the negative side
    COPLANAR,  // all vertices on (or very near) the plane
    STRADDLE   // vertices on both sides
};

/**
 * Determine which side of a plane a triangle is on.
 * The plane is defined as: normal · P + d = 0
 */
inline PlaneSide classifyTriangle(const MeshTriangle& tri, const Vetor& normal,
                                  double d, double tolerance = 1e-9) {
    int frontCount = 0, backCount = 0, onCount = 0;
    const Ponto* verts[3] = {&tri.v0, &tri.v1, &tri.v2};
    for (int i = 0; i < 3; ++i) {
        double dist = normal.getX() * verts[i]->getX() +
                      normal.getY() * verts[i]->getY() +
                      normal.getZ() * verts[i]->getZ() + d;
        if (dist > tolerance)
            frontCount++;
        else if (dist < -tolerance)
            backCount++;
        else
            onCount++;
    }

    if (frontCount > 0 && backCount > 0) return PlaneSide::STRADDLE;
    if (frontCount > 0) return PlaneSide::FRONT;
    if (backCount > 0) return PlaneSide::BACK;
    return PlaneSide::COPLANAR;
}

/**
 * Split a triangle by a plane, producing fragments for each side.
 *
 * The splitter triangle and any coplanar triangles should NOT be passed
 * here — use this only for triangles classified as STRADDLE.
 *
 * @param tri      The triangle to split.
 * @param normal   Plane normal.
 * @param d        Plane constant (normal·P + d = 0).
 * @param frontOut Output vector for front-side fragments (1 triangle).
 * @param backOut  Output vector for back-side fragments (1 or 2 triangles).
 */
inline void splitTriangle(const MeshTriangle& tri, const Vetor& normal,
                          double d, std::vector<MeshTriangle>& frontOut,
                          std::vector<MeshTriangle>& backOut) {
    const Ponto* verts[3] = {&tri.v0, &tri.v1, &tri.v2};
    double dists[3];
    int side[3];  // 1 = front, -1 = back, 0 = on plane

    for (int i = 0; i < 3; ++i) {
        dists[i] = normal.getX() * verts[i]->getX() +
                   normal.getY() * verts[i]->getY() +
                   normal.getZ() * verts[i]->getZ() + d;
        if (dists[i] > 1e-9)
            side[i] = 1;
        else if (dists[i] < -1e-9)
            side[i] = -1;
        else
            side[i] = 0;
    }

    // Find the isolated vertex (the one alone on its side)
    int isolated = -1;
    for (int i = 0; i < 3; ++i) {
        int next = (i + 1) % 3;
        int prev = (i + 2) % 3;
        if (side[i] != 0 && side[next] != side[i] && side[prev] != side[i]) {
            isolated = i;
            break;
        }
    }

    // Should always find an isolated vertex for a straddling triangle
    if (isolated < 0) {
        // Fallback: classify by majority
        int frontCount = 0, backCount = 0;
        for (int i = 0; i < 3; ++i) {
            if (side[i] == 1) frontCount++;
            if (side[i] == -1) backCount++;
        }
        if (frontCount > backCount)
            frontOut.push_back(tri);
        else
            backOut.push_back(tri);
        return;
    }

    int next = (isolated + 1) % 3;
    int prev = (isolated + 2) % 3;

    // Compute intersection points along edges that cross the plane
    // t = d_isolated / (d_isolated - d_other)
    double t1 = dists[isolated] / (dists[isolated] - dists[next]);
    double t2 = dists[isolated] / (dists[isolated] - dists[prev]);

    Ponto p1 = *verts[isolated] + (*verts[next] - *verts[isolated]) * t1;
    Ponto p2 = *verts[isolated] + (*verts[prev] - *verts[isolated]) * t2;

    bool isolatedOnFront = (side[isolated] == 1);

    MeshTriangle frag;
    frag.normal = tri.normal;
    frag.object = tri.object;

    if (isolatedOnFront) {
        // Isolated vertex on front, two on back
        // Front: triangle(isolated, p1, p2)
        frag.v0 = *verts[isolated];
        frag.v1 = p1;
        frag.v2 = p2;
        frontOut.push_back(frag);

        // Back: quadrilateral(next, prev, p2, p1) → two triangles
        frag.v0 = p1;
        frag.v1 = *verts[next];
        frag.v2 = *verts[prev];
        backOut.push_back(frag);

        frag.v0 = p1;
        frag.v1 = *verts[prev];
        frag.v2 = p2;
        backOut.push_back(frag);
    } else {
        // Isolated vertex on back, two on front
        // Back: triangle(isolated, p1, p2)
        frag.v0 = *verts[isolated];
        frag.v1 = p1;
        frag.v2 = p2;
        backOut.push_back(frag);

        // Front: quadrilateral(next, prev, p2, p1) → two triangles
        frag.v0 = p1;
        frag.v1 = *verts[next];
        frag.v2 = *verts[prev];
        frontOut.push_back(frag);

        frag.v0 = p1;
        frag.v1 = *verts[prev];
        frag.v2 = p2;
        frontOut.push_back(frag);
    }
}

/**
 * Pick the best triangle to use as the splitting plane.
 * Evaluates each triangle's plane and picks the one that best balances
 * the front/back split (minimizes |frontCount - backCount|).
 * Ties are broken by minimizing straddle count.
 *
 * Returns the index of the chosen triangle.
 */
inline int pickSplitterTriangle(const std::vector<MeshTriangle>& triangles) {
    if (triangles.empty()) return -1;
    if (triangles.size() == 1) return 0;

    int bestIdx = 0;
    int bestBalance = std::numeric_limits<int>::max();
    int bestStraddle = std::numeric_limits<int>::max();

    size_t n = triangles.size();
    // Evaluate a subset to keep O(N²) manageable
    int step = std::max(1, (int)n / 20);
    if (step < 1) step = 1;

    for (size_t i = 0; i < n; i += step) {
        const MeshTriangle& tri = triangles[i];
        // Plane equation: N · X + d = 0
        const Vetor& N = tri.normal;
        double d = -(N.getX() * tri.v0.getX() + N.getY() * tri.v0.getY() +
                     N.getZ() * tri.v0.getZ());

        int frontCount = 0, backCount = 0, straddleCount = 0;
        for (size_t j = 0; j < n; ++j) {
            if (i == j) continue;
            PlaneSide side = classifyTriangle(triangles[j], N, d);
            if (side == PlaneSide::FRONT)
                frontCount++;
            else if (side == PlaneSide::BACK)
                backCount++;
            else if (side == PlaneSide::STRADDLE)
                straddleCount++;
            // COPLANAR: does not affect balance
        }

        int balance = std::abs(frontCount - backCount);
        if (balance < bestBalance ||
            (balance == bestBalance && straddleCount < bestStraddle)) {
            bestBalance = balance;
            bestStraddle = straddleCount;
            bestIdx = (int)i;
        }
    }
    return bestIdx;
}

/**
 * Recursively build a BSP tree over mesh triangles.
 *
 * Each internal node picks one triangle's plane as the splitting plane.
 * Straddling triangles are SPLIT into fragments, one for each child.
 * The splitter and any coplanar triangles are stored at the node.
 *
 * @param triangles      Triangles to build the tree from (will be consumed).
 * @param leafThreshold  Max triangles per leaf before forcing a split.
 */
inline std::unique_ptr<MeshBSPNode> buildMeshBSP(
    std::vector<MeshTriangle> triangles, int leafThreshold = 8) {
    auto node = std::make_unique<MeshBSPNode>();

    // Leaf condition: small enough triangle count
    if ((int)triangles.size() <= leafThreshold) {
        node->isLeaf = true;
        node->triangles = std::move(triangles);
        return node;
    }

    // Pick the best triangle to use as the splitting plane
    int splitIdx = pickSplitterTriangle(triangles);
    if (splitIdx < 0) {
        node->isLeaf = true;
        node->triangles = std::move(triangles);
        return node;
    }

    const MeshTriangle& splitTri = triangles[splitIdx];
    const Vetor& N = splitTri.normal;
    double d = -(N.getX() * splitTri.v0.getX() + N.getY() * splitTri.v0.getY() +
                 N.getZ() * splitTri.v0.getZ());

    node->isLeaf = false;
    node->splitNormal = N;
    node->splitD = d;

    // Classify and distribute triangles
    std::vector<MeshTriangle> frontTris, backTris, nodeTris;
    // Reserve capacity for typical distribution
    frontTris.reserve(triangles.size() / 2);
    backTris.reserve(triangles.size() / 2);
    nodeTris.reserve(16);

    for (size_t i = 0; i < triangles.size(); ++i) {
        if ((int)i == splitIdx) {
            // Splitter goes to this node
            nodeTris.push_back(triangles[i]);
            continue;
        }

        PlaneSide side = classifyTriangle(triangles[i], N, d);

        switch (side) {
            case PlaneSide::FRONT:
                frontTris.push_back(std::move(triangles[i]));
                break;
            case PlaneSide::BACK:
                backTris.push_back(std::move(triangles[i]));
                break;
            case PlaneSide::COPLANAR:
                nodeTris.push_back(std::move(triangles[i]));
                break;
            case PlaneSide::STRADDLE:
                // Split the triangle into front and back fragments
                splitTriangle(triangles[i], N, d, frontTris, backTris);
                break;
        }
    }

    // Degenerate split: all triangles ended up on one side → make leaf
    // if (frontTris.empty() || backTris.empty()) {
    //     node->isLeaf = true;
    //     // Move all triangles back into the node
    //     node->triangles.clear();
    //     node->triangles.reserve(frontTris.size() + backTris.size() +
    //                             nodeTris.size());
    //     for (auto& t : frontTris) node->triangles.push_back(std::move(t));
    //     for (auto& t : backTris) node->triangles.push_back(std::move(t));
    //     for (auto& t : nodeTris) node->triangles.push_back(std::move(t));
    //     return node;
    // }

    // Store node triangles (splitter + coplanar)
    node->triangles = std::move(nodeTris);

    // Recurse
    node->front = buildMeshBSP(std::move(frontTris), leafThreshold);
    node->back = buildMeshBSP(std::move(backTris), leafThreshold);

    return node;
}

/**
 * Traverses the BSP strictly in front-to-back order using the
 * splitting planes.  The near child (the side the ray origin is on) is visited
 * first; the far child is visited only if the ray crosses the splitting plane
 * AND closestT is still beyond tSplit (the plane intersection distance).
 *
 * Each stack entry carries the ray's t-range for that subtree:
 *   tMin = nearest possible intersection (for pruning)
 *   tMax = farthest allowed distance (e.g. distance to light)
 * These are derived purely from ancestor splitting-plane distances.
 *
 * Returns true if any intersection is found (the closest one in the
 * ray's range is stored in the output parameters).
 *
 * @param node          Current BSP node to traverse.
 * @param rayOrigin     Ray origin point.
 * @param rayDir        Ray direction vector (should be normalized).
 * @param maxT          Maximum allowed distance along the ray.
 * @param closestT      Output: closest intersection distance found.
 * @param closestNormal Output: normal at the closest intersection.
 * @param hitObject     Output: pointer to the ObjectData of the hit triangle.
 * @param skipObject    Optional: pointer to an ObjectData to skip (e.g. for
 * shadow rays).
 */

/** Counters for dynamic BSP traversal stats (updated atomically). */
struct BSPCounters {
    std::atomic<uint64_t> nodeVisits{0};
    std::atomic<uint64_t> rayTests{0};
    void reset() {
        nodeVisits = 0;
        rayTests = 0;
    }
};

inline bool intersectMeshBSP(MeshBSPNode* node, const Ponto& rayOrigin,
                             const Vetor& rayDir, double maxT, double& closestT,
                             Vetor& closestNormal, ObjectData*& hitObject,
                             ObjectData* skipObject = nullptr,
                             BSPCounters* counters = nullptr) {
    if (!node) return false;
    if (counters) counters->rayTests++;

    bool found = false;

    struct StackEntry {
        MeshBSPNode* node;
        double tMin;  // nearest possible distance for this subtree
        double tMax;  // farthest allowed distance
    };
    std::vector<StackEntry> stack;
    stack.reserve(64);

    stack.push_back({node, 0, maxT});

    while (!stack.empty()) {
        if (counters) counters->nodeVisits++;

        StackEntry entry = stack.back();
        stack.pop_back();

        MeshBSPNode* cur = entry.node;
        double tMin = entry.tMin;
        double tMax = entry.tMax;

        // Prune: nothing in this range can be closer than current closest
        if (tMin >= closestT) continue;

        // --- Test triangles stored at this node ---
        for (const auto& tri : cur->triangles) {
            if (skipObject && tri.object == skipObject) continue;

            const Vetor& normal = tri.normal;
            double dotNorm = rayDir.dot(normal);
            if (std::fabs(dotNorm) < 1e-12) continue;

            double t = (tri.v0 - rayOrigin).dot(normal) / dotNorm;

            if (t > 0 && t < closestT && t < maxT) {
                Ponto P = rayOrigin + rayDir * t;

                // Inside-triangle test (edge cross products)
                Vetor C;
                C = (tri.v1 - tri.v0).cross(P - tri.v0);
                if (normal.dot(C) < 0) continue;
                C = (tri.v2 - tri.v1).cross(P - tri.v1);
                if (normal.dot(C) < 0) continue;
                C = (tri.v0 - tri.v2).cross(P - tri.v2);
                if (normal.dot(C) < 0) continue;

                closestT = t;
                closestNormal = normal;
                hitObject = tri.object;
                found = true;
            }
        }

        if (cur->isLeaf) continue;

        // --- Internal node: front-to-back ordering via splitting plane ---
        double signedDist = cur->splitNormal.getX() * rayOrigin.getX() +
                            cur->splitNormal.getY() * rayOrigin.getY() +
                            cur->splitNormal.getZ() * rayOrigin.getZ() +
                            cur->splitD;

        double denom = cur->splitNormal.dot(rayDir);
        bool startOnFront = (signedDist >= 0);

        MeshBSPNode* nearChild =
            startOnFront ? cur->front.get() : cur->back.get();
        MeshBSPNode* farChild =
            startOnFront ? cur->back.get() : cur->front.get();

        double tSplit = 0.0;
        bool rayCrosses = false;

        if (std::fabs(denom) > 1e-12) {
            tSplit = -signedDist / denom;
            rayCrosses = (tSplit > 0 && tSplit < tMax);
        }

        // Push far child first (LIFO → near child processed first)
        // Only push if ray crosses the plane AND far side is not beyond
        // closestT
        if (farChild && rayCrosses && tSplit < closestT) {
            stack.push_back({farChild, std::max(tMin, tSplit), tMax});
        }

        // Push near child (clipped by tSplit if ray crosses)
        if (nearChild) {
            double nearTMax = rayCrosses ? std::min(tMax, tSplit) : tMax;
            if (tMin < nearTMax) {
                stack.push_back({nearChild, tMin, nearTMax});
            }
        }
    }

    return found;
}

/**
 * Free the mesh BSP tree.
 * (Unique_ptr handles this automatically; this is a convenience wrapper.)
 */
inline void deleteMeshBSP(MeshBSPNode* node) { delete node; }

// ── BSP Statistics ──────────────────────────────────────────────────────────

#include <iomanip>

/** Summary of the BSP tree's static properties. */
struct BSPTreeInfo {
    int height = 0;
    int totalNodes = 0;
    int internalNodes = 0;
    int leafNodes = 0;
    int totalTriangles = 0;
    int minLeafDepth = 0;
    int maxLeafDepth = 0;
    double avgLeafDepth = 0.0;
    int minTrianglesPerLeaf = 0;
    int maxTrianglesPerLeaf = 0;
    double avgTrianglesPerLeaf = 0.0;
};

// Recursive helper for computeBSPTreeInfo
inline void computeBSPStatsRec(const MeshBSPNode* node, int depth, int& height,
                               int& totalNodes, int& internalNodes,
                               int& leafNodes, int& totalTriangles,
                               int& minLeafDepth, int& maxLeafDepth,
                               double& sumLeafDepth, int& leafCountForAvg,
                               int& minTris, int& maxTris, double& sumTris,
                               int& triCountForAvg) {
    if (!node) return;
    totalNodes++;
    totalTriangles += (int)node->triangles.size();

    if (node->isLeaf) {
        leafNodes++;
        if (leafNodes == 1) {
            minLeafDepth = maxLeafDepth = depth;
        } else {
            if (depth < minLeafDepth) minLeafDepth = depth;
            if (depth > maxLeafDepth) maxLeafDepth = depth;
        }
        sumLeafDepth += depth;
        leafCountForAvg++;

        int nt = (int)node->triangles.size();
        if (triCountForAvg == 0) {
            minTris = maxTris = nt;
        } else {
            if (nt < minTris) minTris = nt;
            if (nt > maxTris) maxTris = nt;
        }
        sumTris += nt;
        triCountForAvg++;
    } else {
        if (depth > height) height = depth;
        internalNodes++;
        computeBSPStatsRec(
            node->front.get(), depth + 1, height, totalNodes, internalNodes,
            leafNodes, totalTriangles, minLeafDepth, maxLeafDepth, sumLeafDepth,
            leafCountForAvg, minTris, maxTris, sumTris, triCountForAvg);
        computeBSPStatsRec(
            node->back.get(), depth + 1, height, totalNodes, internalNodes,
            leafNodes, totalTriangles, minLeafDepth, maxLeafDepth, sumLeafDepth,
            leafCountForAvg, minTris, maxTris, sumTris, triCountForAvg);
    }
}

/** Compute static properties of the BSP tree. */
inline BSPTreeInfo computeBSPTreeInfo(const MeshBSPNode* root) {
    BSPTreeInfo info;
    if (!root) return info;

    int height = 0, totalNodes = 0, internalNodes = 0, leafNodes = 0;
    int totalTriangles = 0;
    int minLeafDepth = 0, maxLeafDepth = 0;
    double sumLeafDepth = 0;
    int leafCountForAvg = 0;
    int minTris = 0, maxTris = 0;
    double sumTris = 0;
    int triCountForAvg = 0;

    computeBSPStatsRec(root, 0, height, totalNodes, internalNodes, leafNodes,
                       totalTriangles, minLeafDepth, maxLeafDepth, sumLeafDepth,
                       leafCountForAvg, minTris, maxTris, sumTris,
                       triCountForAvg);

    info.height = height;
    info.totalNodes = totalNodes;
    info.internalNodes = internalNodes;
    info.leafNodes = leafNodes;
    info.totalTriangles = totalTriangles;
    info.minLeafDepth = minLeafDepth;
    info.maxLeafDepth = maxLeafDepth;
    info.avgLeafDepth =
        leafCountForAvg > 0 ? sumLeafDepth / leafCountForAvg : 0.0;
    info.minTrianglesPerLeaf = minTris;
    info.maxTrianglesPerLeaf = maxTris;
    info.avgTrianglesPerLeaf =
        triCountForAvg > 0 ? sumTris / triCountForAvg : 0.0;

    return info;
}

/** Print a formatted summary of BSP tree info and traversal counters. */
inline void printBSPStats(const BSPTreeInfo& info,
                          const BSPCounters& counters) {
    uint64_t totalRays = counters.rayTests.load();
    uint64_t totalVisits = counters.nodeVisits.load();

    std::cout << "\n══════════════════ BSP Tree Stats ══════════════════\n";
    std::cout << "  Tree structure:\n";
    std::cout << "    Height              : " << info.height << "\n";
    std::cout << "    Total nodes         : " << info.totalNodes << "\n";
    std::cout << "    Internal nodes      : " << info.internalNodes << "\n";
    std::cout << "    Leaf nodes          : " << info.leafNodes << "\n";
    std::cout << "    Total triangles     : " << info.totalTriangles
              << " (incl. fragments)\n";

    std::cout << "  Leaf depths:\n";
    std::cout << "    Min                 : " << info.minLeafDepth << "\n";
    std::cout << "    Max                 : " << info.maxLeafDepth << "\n";
    std::cout << "    Avg                 : " << std::fixed
              << std::setprecision(2) << info.avgLeafDepth << "\n";

    std::cout << "  Triangles per leaf:\n";
    std::cout << "    Min                 : " << info.minTrianglesPerLeaf
              << "\n";
    std::cout << "    Max                 : " << info.maxTrianglesPerLeaf
              << "\n";
    std::cout << "    Avg                 : " << std::fixed
              << std::setprecision(2) << info.avgTrianglesPerLeaf << "\n";

    if (totalRays > 0) {
        std::cout << "  Traversal (dynamic):\n";
        std::cout << "    Total ray tests    : " << totalRays << "\n";
        std::cout << "    Total node visits  : " << totalVisits << "\n";
        std::cout << "    Avg nodes / ray    : " << std::fixed
                  << std::setprecision(2) << (double)totalVisits / totalRays
                  << "\n";
    }
    std::cout << "══════════════════════════════════════════════════\n"
              << std::endl;
}

#endif  // MESHBSPHEADER
