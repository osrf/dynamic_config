#include "gtest/gtest.h"
#include "parameter_server/parameter.h"

class ParameterTest : public testing::Test
{
protected:
  typedef std::vector<int> Ints;
  typedef parameter_server::Parameter<Ints> MyParameter;

  class CallbackStruct {
  public:
    CallbackStruct()
    : counter_(0) { };
    bool myCallback(const MyParameter::CallbackResponse& res)
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
    cb = CallbackStructPtr(new CallbackStruct);
    p.subscribe(&CallbackStruct::myCallback, cb);
  }

  Ints intsOfSize(int size)
  {
    Ints ints(size);
    for (int i=0; i<size; ++i)
      ints.push_back(i*10);
    return ints;
  }

  MyParameter p;
  CallbackStructPtr cb;
};

TEST_F(ParameterTest, CopyConstructor) {
  MyParameter q(p);
  EXPECT_EQ(p, q);
}

TEST_F(ParameterTest, OperatorEqual) {
  MyParameter q = p;
  EXPECT_EQ(p, q);
}

TEST_F(ParameterTest, CanGet) {
  p.setValue(intsOfSize(10));
  Ints ints;
  p.getValue(ints);
  EXPECT_EQ(intsOfSize(10), ints);
}

TEST_F(ParameterTest, HasValueFalse) {
  EXPECT_FALSE(p.hasValue());
}

TEST_F(ParameterTest, HasValueTrue) {
  p.setValue(intsOfSize(10));
  EXPECT_TRUE(p.hasValue());
}

TEST_F(ParameterTest, DeleteValueNotSetted) {
  p.deleteValue();
  EXPECT_FALSE(p.hasValue());
}

TEST_F(ParameterTest, DeleteValueSetted) {
  p.setValue(intsOfSize(10));
  p.deleteValue();
  EXPECT_FALSE(p.hasValue());
}

TEST_F(ParameterTest, NotifySubscribers) {
  MyParameter::CallbackResponse res;
  p.notify(res);
  EXPECT_EQ(1, cb->count());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
