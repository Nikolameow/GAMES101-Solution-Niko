#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    if (splitMethod == SplitMethod::NAIVE) {
        printf(" - Generating BVH using naive method...\n\n");
        root = recursiveBuild(primitives);
	}
    else if (splitMethod == SplitMethod::SAH) {
        printf(" - Generating BVH using SAH method...\n\n");
        root = recursiveBuildSAH(primitives);
	}
    

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid()); 
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

bool (*cmp[3])(Object*, Object*) = { [](Object* lhs, Object* rhs) {return lhs->getBounds().Centroid().x < rhs->getBounds().Centroid().x; } , [](Object* lhs, Object* rhs) {return lhs->getBounds().Centroid().y < rhs->getBounds().Centroid().y; },[](Object* lhs, Object* rhs) {return lhs->getBounds().Centroid().z < rhs->getBounds().Centroid().z; } };


BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    int size = objects.size();

    Bounds3 bounds;
    for (int i = 0; i < size; ++i)
        bounds = Union(bounds, objects[i]->getBounds());

    if (size == 1)
    {
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (size == 2)
    {
        node->left = recursiveBuildSAH({ objects[0] });
        node->right = recursiveBuildSAH({ objects[1] });
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        if (size < 12)
        {
            // 选最长轴对半分
            int dim = bounds.maxExtent();
            std::sort(objects.begin(), objects.end(), cmp[dim]);
            auto mid = objects.begin() + (size >> 1);
            auto u = std::vector<Object*>(objects.begin(), mid);
            auto v = std::vector<Object*>(mid, objects.end());
            node->left = recursiveBuildSAH(u);
            node->right = recursiveBuildSAH(v);
            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        else
        {
            int bestDiv = size >> 1;
            int bestDim = 0;
            float minCost = std::numeric_limits<float>::max();
            float parentArea = bounds.SurfaceArea();

            for (int dim = 0; dim < 3; dim++)
            {
                std::sort(objects.begin(), objects.end(), cmp[dim]);

                // 预计算前缀包围盒
                std::vector<Bounds3> prefixBounds(size);
                prefixBounds[0] = objects[0]->getBounds();
                for (int i = 1; i < size; i++)
                    prefixBounds[i] = Union(prefixBounds[i - 1], objects[i]->getBounds());

                // 预计算后缀包围盒
                std::vector<Bounds3> suffixBounds(size);
                suffixBounds[size - 1] = objects[size - 1]->getBounds();
                for (int i = size - 2; i >= 0; i--)
                    suffixBounds[i] = Union(suffixBounds[i + 1], objects[i]->getBounds());

                // 枚举分割点 (在第 i 个和第 i+1 个物体之间分割)
                // 左边 [0, i], 右边 [i+1, size-1]
                for (int i = 0; i < size - 1; i++)
                {
                    float leftArea = prefixBounds[i].SurfaceArea();
                    float rightArea = suffixBounds[i + 1].SurfaceArea();
                    int leftCount = i + 1;
                    int rightCount = size - leftCount;

                    // SAH: C_trav + (S_L / S_parent) * N_L + (S_R / S_parent) * N_R
                    float cost = 0.125f + (leftArea * leftCount + rightArea * rightCount) / parentArea;

                    if (cost < minCost)
                    {
                        minCost = cost;
                        bestDiv = leftCount; // 左子集的物体数
                        bestDim = dim;
                    }
                }
            }

            // 按最佳维度重新排序
            std::sort(objects.begin(), objects.end(), cmp[bestDim]);
            auto mid = objects.begin() + bestDiv;
            auto u = std::vector<Object*>(objects.begin(), mid);
            auto v = std::vector<Object*>(mid, objects.end());
            node->left = recursiveBuildSAH(u);
            node->right = recursiveBuildSAH(v);
            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        return node;
    }
}



Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    if (!node) return {};

    std::array<int, 3> dirIsNeg = {
        ray.direction.x > 0,
        ray.direction.y > 0,
        ray.direction.z > 0
    };

    // 光线未命中包围盒，直接返回
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
        return {};

    // 叶子节点：直接与物体求交
    if (!node->left && !node->right)
        return node->object->getIntersection(ray);

    // 内部节点：递归左右子树，取更近的交点
    Intersection leftHit = getIntersection(node->left, ray);
    Intersection rightHit = getIntersection(node->right, ray);

    if (leftHit.happened && rightHit.happened)
        return leftHit.distance < rightHit.distance ? leftHit : rightHit;
    else if (leftHit.happened)
        return leftHit;
    else
        return rightHit;
}