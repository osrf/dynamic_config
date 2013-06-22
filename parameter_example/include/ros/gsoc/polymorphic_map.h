#include <stdexcept>
#include <boost/any.hpp>
#include <map>


namespace ros {

  namespace gsoc {

typedef std::runtime_error InvalidParameterTypeException; 

template <
  typename Key,
  typename Comparator = std::less<Key>,
  template < typename U > class Alloc = std::allocator
  >
class PolymorphicMap 
{	
  typedef Key key_type;
  typedef boost::any mapped_type;

  typedef Alloc< std::pair<const Key, mapped_type> > allocator_type;
  typedef typename std::map< Key, boost::any, Comparator, allocator_type > prop_map_type;
  typedef typename prop_map_type::value_type value_type;
  typedef typename prop_map_type::iterator iterator;
  typedef typename prop_map_type::const_iterator const_iterator;
  typedef typename prop_map_type::size_type size_type;

private:

  mutable prop_map_type prop_map;

public:

  template < typename val_type >
  void set(const key_type& k, const val_type& v)
  {
    val_type val(v);
    prop_map[k] = val;
  }

  template <typename val_type >
  void get(const key_type& k, val_type& v)
  {
    val_type* val = boost::any_cast<val_type>(&prop_map[k]);
    if (!val) throw InvalidParameterTypeException("Wrong type");
    v = *val;
  }

  bool has(const key_type& k) const 
  {
    iterator it = prop_map.find(k);
    return it != prop_map.end();
  }

  void del(const key_type& k)
  {
    iterator it = prop_map.find(k);
    if (it != prop_map.end()) {
      prop_map.erase(it);
    }
  }
};


  } // gsoc

} // ros
