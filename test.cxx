#include <iostream>
#include <cassert>
#include <algorithm>
#include <functional>
#include <iterator>
#include "packed_vector.hxx"


#ifdef _MSC_VER
typedef packed_vector<unsigned char, 3, unsigned __int64> vec_type;
#else
typedef packed_vector<unsigned char, 3, unsigned long> vec_type;
#endif


void
test_1 ()
{
  std::cout
    << "bits: " << vec_type::bits << "\n"
    << "bit_mask: 0x" << std::hex << vec_type::bit_mask << "\n"
    << std::dec;
}


void
test_2 ()
{
  vec_type v;
}


void
test_3 ()
{
  vec_type v (10);
}


void
test_4 ()
{
  vec_type v (10, 1);
  assert (v[9] == 1);

  vec_type const & u = v;
  assert (u[1] == 1);
}


void
test_5 ()
{
  vec_type v (21, 0);
  v[19] = 2;
  assert (v[19] == 2);
}


void
test_6 ()
{
  vec_type v (10, 0);
  vec_type::const_reference cref = v[9];
  vec_type::const_reference cref2 = cref;
  assert (cref == 0);
  v[9] = 2;
  assert (v[9] == 2);
  assert (cref == 2);
  assert (cref == cref2);
}


void
test_7 ()
{
  vec_type v (10, 0);
  vec_type::reference ref = v[9];
  vec_type::reference ref2 = ref;
  assert (ref == 0);
  ref = 2;
  assert (v[9] == 2);
  assert (ref == 2);
  assert (ref == ref2);
}


void
test_8 ()
{
  vec_type v (100, 4);

  for (size_t i = 0; i < v.size (); ++i)
    assert (v[i] == 4);
}


void
test_9 ()
{
  packed_vector<int, 3> v (21);

  v[18] = 3;
  v[19] = -3;

  assert (v[19] == -3);
  std::cout << "v[19]: " << v[19] << "\n";

  assert (v[18] == 3);
  std::cout << "v[18]: " << v[18] << "\n";
}


void
test_10 ()
{
  vec_type u (10, 2);
  vec_type v (u.begin (), u.end ());

  for (size_t i = 0; i < v.size (); ++i)
    {
      vec_type::value_type val = v[i];
      assert (val == 2);
    }

  for (vec_type::const_iterator i = v.begin ();  i != v.end (); ++i)
    {
      vec_type::value_type val = *i;
      assert (val == 2);
    }
}


void
test_11 ()
{
  vec_type u (10, 2);
  vec_type v (u.begin (), u.end ());
  for (vec_type::const_reverse_iterator i = v.rbegin ();  i != v.rend (); ++i)
    assert (*i == 2);
}


void
test_12 ()
{
  vec_type v;
  v.resize (100, 1);
  for (vec_type::const_reverse_iterator i = v.rbegin ();  i != v.rend (); ++i)
    assert (*i == 1);
}


void
test_13 ()
{
  vec_type v (100, 2);
  vec_type u (v);
  vec_type w;
  u[0] = 1;
  assert (u < v);
  assert (w < u);
}


void
test_14 ()
{
  vec_type v (100, 2);
  vec_type u (v);
  vec_type w;
  assert (u == v);
  assert (! (u == w));
}


void
test_15 ()
{
  vec_type v (100, 2);

  vec_type::value_type val = *v.insert (v.begin () + 50, 3);
  assert (val == 3);

  for (vec_type::iterator i = v.begin ();  i != v.begin () + 50; ++i)
    assert (*i == 2);

  assert (*(v.begin () + 50) == 3);

  for (vec_type::iterator i = v.begin () + 50 + 1;  i != v.end (); ++i)
    assert (*i == 2);

  vec_type::const_iterator middle = v.erase (v.begin () + 50);
  assert (*middle == 2);
  assert (v.size () == 100);

  for (vec_type::iterator i = v.begin ();  i != v.end (); ++i)
    assert (*i == 2);
}


void
test_16 ()
{
  vec_type v (100, 2);
  vec_type u;

  u.assign (100, 1);
  assert (u.size () == 100);
  assert (std::find_if (u.begin (), u.end (),
			std::bind1st (std::not_equal_to<char>(), 1))
	  == u.end ());

  v.assign (u.begin (), u.end ());
  assert (std::find_if (v.begin (), v.end (),
			std::bind1st (std::not_equal_to<char>(), 1))
	  == v.end ());
}


void
test_17 ()
{
  vec_type v (100, 2);
  vec_type u;

  assert (! v.empty ());
  v.clear ();
  assert (v.empty ());
  assert (u.empty ());
}


void
test_18 ()
{
  vec_type v (100, 2);
  vec_type u;

  std::cout << "v.capacity (): " << v.capacity () << "\n"
            << "v.max_size (): " << v.max_size () << std::endl;

  std::cout << "u.capacity (): " << u.capacity () << "\n";
  u.reserve (100);
  std::cout << "u.capacity (): " << u.capacity () << std::endl;
}


void
test_19 ()
{
  typedef packed_vector<signed char, 2, unsigned short> vec_type;

  bool threw = false;
  vec_type u;
  int val = -1;

  try
    {
      val = u.at (10);
    }
  catch (std::out_of_range const & e)
    {
      threw = true;
    }
  assert (threw && val == -1);

  u.resize (11, -2);
  val = 0;
  threw = false;

  try
    {
      val = u.at (10);
    }
  catch (std::out_of_range const & e)
    {
      threw = true;
    }
  assert (! threw && val == -2);
}


void
test_20 ()
{
  vec_type v (10, 7);

  assert (v.front () == v.back ());
  assert (v.front () == 7);

  v.front () = 1;
  assert (v.front () == 1);

  v.back () = 2;
  assert (v.back () == 2);

  vec_type const & u = v;

  assert (u.front () == 1);
  assert (u.back () == 2);
}


void
test_21 ()
{
  vec_type v (100, 2);
  vec_type u;

  u.swap (v);
  assert (u.size () == 100);
  assert (v.empty ());
  assert (std::find_if (u.begin (), u.end (),
			std::bind1st (std::not_equal_to<char>(), 2))
          == u.end ());

  v.swap (u);
  assert (std::find_if (v.begin (), v.end (),
			std::bind1st (std::not_equal_to<char>(), 2))
	  == v.end ());
}


void
test_22 ()
{
  typedef packed_vector<signed char, 2, unsigned short> vec_type;

  bool threw = false;
  vec_type u;
  vec_type const & v = u;
  int val = -1;

  try
    {
      val = v.at (10);
    }
  catch (std::out_of_range const & e)
    {
      threw = true;
    }
  assert (threw && val == -1);

  u.resize (11, -2);
  val = 0;
  threw = false;

  try
    {
      val = v.at (10);
    }
  catch (std::out_of_range const & e)
    {
      threw = true;
    }
  assert (! threw && val == -2);
}


void
test_23 ()
{
  vec_type u (100, 3);
  vec_type const & v = u;

  assert (u.end () - u.begin ()
          == static_cast<vec_type::iterator::difference_type> (u.size ()));
  assert (v.begin () - v.end ()
          == static_cast<vec_type::iterator::difference_type> (-u.size ()));
}


void
test_24 ()
{
  vec_type u;
  vec_type const & v = u;

  u.push_back (1);
  assert (u.size () == 1);
  assert (v.front () == 1);

  u.push_back (2);
  assert (v.back () == 2);

  u.push_back (3);
  assert (*v.rbegin () == 3);

  u.pop_back ();
  assert (v.back () == 2);

  u.pop_back ();
  assert (v.back () == 1);

  u.pop_back ();
  assert (v.empty ());
}


void
test_25 ()
{
  vec_type u;
  u.insert (u.begin (), 10, 1);
  assert (u.size () == 10);
  assert (std::find_if (u.begin (), u.end (),
			std::bind1st (std::not_equal_to<char>(), 1))
          == u.end ());

  u.insert (u.begin () + 5, 10, 2);
  std::ostream_iterator<int> osit (std::cout, " ");
  std::copy (u.begin (), u.end (), osit);
  std::cout << std::endl;
  assert (std::find_if (u.begin () + 5, u.begin () + 5 + 10,
			std::bind1st (std::not_equal_to<char>(), 2))
          == u.begin () + 5 + 10);
}



int
main ()
{
  test_1 ();
  test_2 ();
  test_3 ();
  test_4 ();
  test_5 ();
  test_6 ();
  test_7 ();
  test_8 ();
  test_9 ();
  test_10 ();
  test_11 ();
  test_12 ();
  test_13 ();
  test_14 ();
  test_15 ();
  test_16 ();
  test_17 ();
  test_18 ();
  test_19 ();
  test_20 ();
  test_21 ();
  test_22 ();
  test_23 ();
  test_24 ();
  test_25 ();
}
