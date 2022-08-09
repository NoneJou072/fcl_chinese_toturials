#include <gtest/gtest.h>

#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "test_fcl_utility.h"
#include "eigen_matrix_compare.h"
#include "fcl_resources/config.h"

using namespace fcl;

// 最大接触点数，即 int 类型的极限值
int num_max_contacts = std::numeric_limits<int>::max();
// 接触使能: false -> 只对碰撞与否感兴趣
//          true -> 计算接触信息
bool enable_contact = true;
// 获取更多信息
bool verbose = true;

template <typename S>
bool collide_Test_OBB(const Transform3<S>& tf,
                      const std::vector<Vector3<S>>& vertices1, const std::vector<Triangle>& triangles1,
                      const std::vector<Vector3<S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method, bool verbose);

template<typename BV, typename TraversalNode>
void distance_Test_Oriented(const Transform3<typename BV::S>& tf,
                            const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                            const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method,
                            int qsize,
                            test::DistanceRes<typename BV::S>& distance_result,
                            bool verbose = true);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename S>
void test_mesh_distance()
{
  std::vector<Vector3<S>> p1, p2;
  std::vector<Triangle> t1, t2;

  test::loadOBJFile(TEST_RESOURCES_DIR"/env.obj", p1, t1);
  test::loadOBJFile(TEST_RESOURCES_DIR"/rob.obj", p2, t2);

  aligned_vector<Transform3<S>> transforms; // t0
  S extents[] = {-3000, -3000, 0, 3000, 3000, 3000};

  std::size_t n = 10;

  test::generateRandomTransforms(extents, transforms, n);

  test::DistanceRes<S> res;
  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    // 碰撞检测
    collide_Test_OBB(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    // 距离检测
    distance_Test_Oriented<RSS<S>, detail::MeshDistanceTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, 2, res, verbose);
    std::cout << "====================== " << std::endl;
  }
}
/////////////////////////////////////////////        TEST         //////////////////////////////////////////////////
GTEST_TEST(FCL_DISTANCE, mesh_distance)
{
  test_mesh_distance<double>();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename BV, typename TraversalNode>
void distance_Test_Oriented(const Transform3<typename BV::S>& tf,
                            const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                            const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method,
                            int qsize,
                            test::DistanceRes<typename BV::S>& distance_result,
                            bool verbose)
{
  using S = typename BV::S;

  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));


  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  DistanceResult<S> local_result;
  TraversalNode node;
  if(!initialize(node, (const BVHModel<BV>&)m1, tf, (const BVHModel<BV>&)m2, Transform3<S>::Identity(), DistanceRequest<S>(true), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  distance(&node, nullptr, qsize);

  // 点在相对于全局坐标系的局部坐标系中
  // 获取局部坐标系中最近的两个点
  Vector3<S> p1 = local_result.nearest_points[0];
  Vector3<S> p2 = local_result.nearest_points[1];

  distance_result.distance = local_result.min_distance;
  distance_result.p1 = p1;
  distance_result.p2 = p2;

  if(verbose)
  {
    std::cout << "distance " << local_result.min_distance << std::endl; // 打印最小距离

    std::cout << p1[0] << " " << p1[1] << " " << p1[2] << std::endl; // 打印两个最近点的坐标
    std::cout << p2[0] << " " << p2[1] << " " << p2[2] << std::endl; 
    std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 碰撞测试
template <typename S>
bool collide_Test_OBB(const Transform3<S>& tf,
                      const std::vector<Vector3<S>>& vertices1, const std::vector<Triangle>& triangles1,
                      const std::vector<Vector3<S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method, bool verbose)
{
  // 构建两棵 BVH树
  BVHModel<OBB<S>> m1;
  BVHModel<OBB<S>> m2;
  m1.bv_splitter.reset(new detail::BVSplitter<OBB<S>>(split_method));
  m2.bv_splitter.reset(new detail::BVSplitter<OBB<S>>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  CollisionResult<S> local_result;
  detail::MeshCollisionTraversalNodeOBB<S> node;
  // 节点初始化, 将两棵 BVH树 和它们之间对应关系保存在 node 中
  // 注：node 取的是 result 的地址，在后续的 collide() 中，result 会随 node 更改
  if(!detail::initialize(node, m1, tf, m2, Transform3<S>::Identity(),
                 CollisionRequest<S>(), local_result))
    std::cout << "initialize error" << std::endl;
  // 统计使能
  node.enable_statistics = verbose;

  // collide(&node);
  detail::collide(&node);

  // 打印统计信息
  if(local_result.numContacts() > 0){
    if(verbose){
      // numContacts -> 接触点的数目
      std::cout << "in collision " << local_result.numContacts() << ": " << std::endl;
      // num_bv_tests -> 已测试的 BV 数目
      // num_leaf_tests -> 已测试的叶子节点数目
      std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    }
    return true;
  }
  else{
    if(verbose){
      std::cout << "collision free " << std::endl;
      std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    }
    return false;
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
