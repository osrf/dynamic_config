#include "gtest/gtest.h"
#include "parameter_server/parameters_server.h"

class ParametersServerTest : public testing::Test
{
protected:
  typedef int Value;
  typedef parameter_server::ParametersServer<Value> ParamServer;

  class CallbackStruct {
    public:
      CallbackStruct()
      : counter_(0) { };
      bool myCallback(const ParamServer::Response& res)
      { 
        ++counter_;
        return true; 
      }
      int count()
      { return counter_; }
    private:
      int counter_;
    };
  typedef boost::shared_ptr<CallbackStruct> CallbackStructPtr;


  virtual void SetUp()
  {
    srv.set("p1", 1);
  }

  ParamServer srv;
};

TEST_F(ParametersServerTest, CanRemoveParameter) {
  srv.remove("param1");
}

TEST_F(ParametersServerTest, CheckParameterExist) {
  EXPECT_TRUE(srv.has("p1"));
  EXPECT_FALSE(srv.has("no"));
}

TEST_F(ParametersServerTest, GetExistentParameter_shouldReturnTrue) {
  int p1;
  EXPECT_TRUE(srv.get("p1", p1));
  EXPECT_EQ(1, p1);
}

TEST_F(ParametersServerTest, GetNotExistentParameter_shouldReturnFalse) {
  int no;
  EXPECT_FALSE(srv.get("no", no));
}

TEST_F(ParametersServerTest, SetParameter_shouldNotifySubscribers) {
  {
    CallbackStructPtr cb(new CallbackStruct);
    srv.subscribe("p1", &CallbackStruct::myCallback, cb);
    srv.set("p1", 20);
    EXPECT_EQ(1, cb->count());
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
