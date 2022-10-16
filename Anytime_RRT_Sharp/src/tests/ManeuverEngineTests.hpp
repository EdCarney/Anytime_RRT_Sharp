#include <gtest/gtest.h>
#include "../Geometry3D.hpp"
#include "../ManeuverEngine.hpp"

#pragma region ObstacleIntersection

TEST(ObstacleIntersection, Bug_001_Forward)
{
    State start(87.7206, 102.164, 55.9721, 5.90294, 0.405405);
    State final(107.423, 66.3749, 28.3135, 1.18302, -0.019936);

    Point rectMinP(84.85440264061208, 78.30774495501349, 0.0);
    Point rectMaxP(91.12307617710908, 88.17614485668203, 78.60665605131811);
    Rectangle r(rectMinP, rectMaxP);

    ManeuverEngine::maneuverType = Dubins3d;
    auto path = ManeuverEngine::generatePath(start, final);

    bool unsafe = false;

    for (auto p : path)
        if (r.intersects(p))
            unsafe = true;

    ASSERT_TRUE(unsafe);
}

TEST(ObstacleIntersection, Bug_001_Reverse)
{
    State start(87.7206, 102.164, 55.9721, 5.90294, 0.405405);
    State final(107.423, 66.3749, 28.3135, 1.18302, -0.019936);

    Point rectMinP(84.85440264061208, 78.30774495501349, 0.0);
    Point rectMaxP(91.12307617710908, 88.17614485668203, 78.60665605131811);
    Rectangle r(rectMinP, rectMaxP);

    auto path = ManeuverEngine::generatePath(final, start);

    bool unsafe = false;

    for (auto p : path)
        if (r.intersects(p))
            unsafe = true;

    ASSERT_TRUE(unsafe);
}