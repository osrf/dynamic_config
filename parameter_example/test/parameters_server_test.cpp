#include "gtest/gtest.h"
#include "parameter_server/parameters_server.h"

class ParametersServerTest : public testing::Test
{
protected:
  typedef std::vector<int> Vector;

  class MockNotifier
  {
  public:
    bool call(const std::string& name, const std::string& subs_id, int event)
    { 
      map[name][subs_id] = event;
    }

    int lastEvent(const std::string& name, const std::string& subs_id)
    {
      return map[name][subs_id];
    }

    typedef std::map<std::string, int> SecondMap;
    typedef std::map<std::string, SecondMap> FirstMap;
    FirstMap map;
  };

  Vector createVector(int size)
  {
    Vector v(size);
    for (int i=0; i<size; ++i) {
      v[i] = i*10;
    }
    return v;
  }

  parameter_server::ParametersServer<Vector, MockNotifier> srv;
};

TEST_F(ParametersServerTest, SetAndGetManyParameters) {
  srv.set("v1", createVector(10));
  srv.set("v2", createVector(20));
  Vector v1, v2;
  srv.get("v1", v1);
  srv.get("v2", v2);
  EXPECT_EQ(v1, createVector(10));
  EXPECT_EQ(v2, createVector(20));
}

TEST_F(ParametersServerTest, GetNonExistentParameter_shouldDoNothing) {
  Vector v;
  Vector vCopy(v);
  srv.get("noExist", v);
  EXPECT_EQ(v, vCopy);
}

TEST_F(ParametersServerTest, canSubscribe) {
  srv.subscribe("p", "subcriber_id");
}

TEST_F(ParametersServerTest, SetNonExistentParameter_shouldNotifySubscribers) {
  srv.subscribe("p", "subs1");
  srv.set("p", createVector(10)); 
  srv.processSubscribersEvents();
  EXPECT_EQ(0, srv.lastEvent("p", "subs1"));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
