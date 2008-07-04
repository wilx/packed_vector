// Copyright (c) 2008, Václav Haisman
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef PACKED_VECTOR_HXX
#define PACKED_VECTOR_HXX

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <iterator>
#include <memory>
#include <limits>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/iterator/reverse_iterator.hpp>
#include <boost/type_traits/is_signed.hpp>
#include <boost/type_traits/is_integral.hpp>
#include <boost/utility/enable_if.hpp>


#ifndef PACKED_VECTOR_TMPL_HEADER
#  define PACKED_VECTOR_TMPL_HEADER                                     \
  template <typename T, size_t Bits, typename StorageUnitType,          \
            typename Allocator>
#else
#  error "PACKED_VECTOR_TMPL_HEADER is already defined!"
#endif

#ifndef PACKED_VECTOR_TMPL_BR
#  define PACKED_VECTOR_TMPL_BR T, Bits, StorageUnitType, Allocator
#else
#  error "PACKED_VECTOR_TMPL_BR is already defined!"
#endif


template <typename T, size_t Bits, typename StorageUnitType = size_t,
          typename Allocator = std::allocator<StorageUnitType> >
class packed_vector;


namespace detail
{

PACKED_VECTOR_TMPL_HEADER
class packed_vector_value_ref;


PACKED_VECTOR_TMPL_HEADER
class packed_vector_iter;


PACKED_VECTOR_TMPL_HEADER
class packed_vector_const_iter;


PACKED_VECTOR_TMPL_HEADER
class packed_vector_value_const_ref
{
public:
  typedef packed_vector<PACKED_VECTOR_TMPL_BR> packed_vector_type;
  typedef T value_type;

  packed_vector_value_const_ref (packed_vector_value_const_ref const & other)
    : vec (other.vec)
    , index (other.index)
  { }

  operator value_type () const;

private:
  packed_vector_value_const_ref ();

  packed_vector_value_const_ref &
  operator = (packed_vector_value_const_ref const &);

  packed_vector_value_const_ref &
  operator = (packed_vector_value_const_ref const &) const;

  packed_vector_value_const_ref (packed_vector_type const * v, size_t i)
    : vec (v)
    , index (i)
  { }

  friend class packed_vector<PACKED_VECTOR_TMPL_BR>;
  friend class packed_vector_value_ref<PACKED_VECTOR_TMPL_BR>;
  friend class packed_vector_const_iter<PACKED_VECTOR_TMPL_BR>;

  packed_vector_type const * const vec;
  size_t const index;
};


PACKED_VECTOR_TMPL_HEADER
class packed_vector_value_ref
{
public:
  typedef packed_vector<PACKED_VECTOR_TMPL_BR> packed_vector_type;
  typedef T value_type;
  typedef packed_vector_value_const_ref<PACKED_VECTOR_TMPL_BR> const_reference;

  packed_vector_value_ref (packed_vector_value_ref const & other)
    : vec (other.vec)
    , index (other.index)
  { }

  packed_vector_value_ref const &
  operator = (value_type) const;

  packed_vector_value_ref &
  operator = (packed_vector_value_ref const &);

  packed_vector_value_ref const &
  operator = (packed_vector_value_ref const &) const;

  operator value_type () const;

  operator const_reference () const
  {
    return const_reference (vec, index);
  }

private:
  packed_vector_value_ref ();

  packed_vector_value_ref (packed_vector_type * v, size_t i)
    : vec (v)
    , index (i)
  { }

  friend class packed_vector<PACKED_VECTOR_TMPL_BR>;
  friend class packed_vector_iter<PACKED_VECTOR_TMPL_BR>;

  packed_vector_type * const vec;
  size_t const index;
};


//
PACKED_VECTOR_TMPL_HEADER
class packed_vector_iter
  : public boost::iterator_facade<
  packed_vector_iter<PACKED_VECTOR_TMPL_BR>
  , packed_vector_value_ref<PACKED_VECTOR_TMPL_BR>
  , std::random_access_iterator_tag
  , packed_vector_value_ref<PACKED_VECTOR_TMPL_BR> >
{
public:
  typedef boost::iterator_facade<
  packed_vector_iter<PACKED_VECTOR_TMPL_BR>
  , packed_vector_value_ref<PACKED_VECTOR_TMPL_BR>
  , std::random_access_iterator_tag
  , packed_vector_value_ref<PACKED_VECTOR_TMPL_BR> >
  base;

  typedef packed_vector<PACKED_VECTOR_TMPL_BR> packed_vector_type;

  packed_vector_iter ()
  { }

protected:
  packed_vector_iter (packed_vector_type * v, size_t i)
    : vec (v)
    , index (i)
  { }

  friend class packed_vector<PACKED_VECTOR_TMPL_BR>;
  friend class packed_vector_const_iter<PACKED_VECTOR_TMPL_BR>;

  friend class boost::iterator_core_access;

  typename base::reference
  dereference () const
  {
    return typename base::reference (vec, index);
  }

  bool
  equal (packed_vector_iter const & other) const
  {
    assert (vec == other.vec);
    return index == other.index;
  }

  void
  increment ()
  {
    assert (index + 1 <= vec->size ());
    index += 1;
  }

  void
  decrement ()
  {
    assert (index != 0);
    index -= 1;
  }

  void
  advance (typename base::difference_type n)
  {
#ifndef NDEBUG
    if (n < 0)
      assert (static_cast<size_t>(-n) <= index);
    else
      assert (index + n <= vec->size ());
#endif // NDEBUG
    index += n;
  }

  typename base::difference_type
  distance_to (packed_vector_iter const & other) const
  {
    assert (vec == other.vec);
    return other.index - index;
  }

  packed_vector_type * vec;
  size_t index;
};


//
PACKED_VECTOR_TMPL_HEADER
class packed_vector_const_iter
  : public boost::iterator_facade<
  packed_vector_const_iter<PACKED_VECTOR_TMPL_BR>
  , packed_vector_value_const_ref<PACKED_VECTOR_TMPL_BR>
  , std::random_access_iterator_tag
  , packed_vector_value_const_ref<PACKED_VECTOR_TMPL_BR> >
{
public:
  typedef boost::iterator_facade<
  packed_vector_const_iter<PACKED_VECTOR_TMPL_BR>
  , packed_vector_value_const_ref<PACKED_VECTOR_TMPL_BR>
  , std::random_access_iterator_tag
  , packed_vector_value_const_ref<PACKED_VECTOR_TMPL_BR> >
  base;

  typedef packed_vector<PACKED_VECTOR_TMPL_BR> packed_vector_type;

  packed_vector_const_iter ()
  { }

  packed_vector_const_iter (packed_vector_iter<PACKED_VECTOR_TMPL_BR>
                            const & iter)
    : vec (iter.vec)
    , index (iter.index)
  { }

protected:
  packed_vector_const_iter (packed_vector_type const * v, size_t i)
    : vec (v)
    , index (i)
  { }

  friend class packed_vector<PACKED_VECTOR_TMPL_BR>;

  friend class boost::iterator_core_access;

  typename base::reference
  dereference () const
  {
    return typename base::reference (vec, index);
  }

  bool
  equal (packed_vector_const_iter const & other) const
  {
    assert (vec == other.vec);
    return index == other.index;
  }

  void
  increment ()
  {
    assert (index + 1 <= vec->size ());
    index += 1;
  }

  void
  decrement ()
  {
    assert (index != 0);
    index -= 1;
  }

  void
  advance (typename base::difference_type n)
  {
#ifndef NDEBUG
    if (n < 0)
      assert (static_cast<size_t>(-n) <= index);
    else
      assert (index + n <= vec->size ());
#endif // NDEBUG
    index += n;
  }

  typename base::difference_type
  distance_to (packed_vector_const_iter const & other) const
  {
    assert (vec == other.vec);
    return other.index - index;
  }

  packed_vector_type const * vec;
  size_t index;
};


template <typename T>
struct enable_non_integral
  : boost::disable_if_c<boost::is_integral<T>::value, T>
{ };


} // namespace detail


//PACKED_VECTOR_TMPL_HEADER
template <typename T, size_t Bits, typename StorageUnitType,
          typename Allocator>
class packed_vector
{
public:
  typedef StorageUnitType storage_unit_type;
  typedef Allocator allocator_type;
  typedef std::vector<storage_unit_type> storage_type;
  typedef typename storage_type::size_type size_type;

  enum BitValues
    {
      bits = Bits,
      storage_unit_bits = sizeof (storage_unit_type) *
#ifdef CHAR_BIT
      CHAR_BIT
#else
      8
#endif // CHAR_BIT
      ,
      elems_per_storage_unit = storage_unit_bits / bits
        + !! (storage_unit_bits % bits),
      complete_elems_per_storage_unit = storage_unit_bits / bits
    };

  static storage_unit_type const bit_mask = (1 << bits) - 1;

  typedef T value_type;

  typedef detail::packed_vector_value_ref<
    PACKED_VECTOR_TMPL_BR>
  reference;

  typedef detail::packed_vector_value_const_ref<
    PACKED_VECTOR_TMPL_BR>
  const_reference;

  typedef typename storage_type::difference_type difference_type;

  typedef detail::packed_vector_iter<PACKED_VECTOR_TMPL_BR> iterator;

  typedef detail::packed_vector_const_iter<PACKED_VECTOR_TMPL_BR>
  const_iterator;

  typedef boost::reverse_iterator<iterator> reverse_iterator;

  typedef boost::reverse_iterator<const_iterator> const_reverse_iterator;

  explicit
  packed_vector (allocator_type const & = allocator_type ())
    : elems (0)
  { }

  explicit
  packed_vector (size_type n, value_type t = value_type (),
                 allocator_type const & = allocator_type ())
    : elems (n), storage (needed_storage_size ())
  {
    std::fill (begin (), end (), t);
  }

  template <typename InputIterator>
  packed_vector (InputIterator b,
                 typename detail::enable_non_integral<InputIterator>::type e,
                 allocator_type const & = allocator_type ())
  {
    size_type n = std::distance (b, e);
    just_resize (n);
    std::copy (b, e, begin ());
  }

  bool
  empty () const
  {
    return elems == 0;
  }

  size_type
  size () const
  {
    return elems;
  }

  size_type
  max_size () const
  {
    return storage.max_size ();
  }

  size_type
  capacity () const
  {
    return storage.capacity () * storage_unit_bits / bits;
  }

  void
  reserve (size_type n)
  {
    storage.reserve (needed_storage_size (n));
  }

  void
  resize (size_type n, value_type val = value_type ())
  {
    size_type const old_size = elems;
    just_resize (n);
    if (elems > old_size)
      std::fill (begin () + old_size, end (), val);
  }

  void
  clear ()
  {
    elems = 0;
    storage.clear ();
  }

  reference
  operator [] (size_type i)
  {
    return reference (this, i);
  }

  const_reference
  operator [] (size_type i) const
  {
    return const_reference (this, i);
  }

  reference
  at (size_type i)
  {
    if (i < elems)
      return reference (this, i);
    else
      throw std::out_of_range ("bad index");
  }

  const_reference
  at (size_type i) const
  {
    if (i < elems)
      return const_reference (this, i);
    else
      throw std::out_of_range ("bad index");
  }

  reference
  front ()
  {
    return reference (this, 0);
  }

  const_reference
  front () const
  {
    return const_reference (this, 0);
  }

  reference
  back ()
  {
    return reference (this, elems - 1);
  }

  const_reference
  back () const
  {
    return const_reference (this, elems - 1);
  }

  void
  push_back (value_type x)
  {
    just_resize (elems + 1);
    set_elem (elems - 1, x);
  }

  void
  pop_back ()
  {
    just_resize (elems - 1);
  }

  void
  swap (packed_vector & other)
  {
    std::swap (elems, other.elems);
    storage.swap (other.storage);
  }

  iterator
  insert (iterator pos, value_type x)
  {
    assert (pos.vec == this);
    just_resize (elems + 1);
    std::copy_backward (pos, end () - 1, end ());
    *pos = x;
    return pos;
  }

  void
  insert (iterator pos, size_type n, value_type x)
  {
    assert (pos.vec == this);
    just_resize (elems + n);
    std::copy_backward (pos, end () - n, end ());
    std::fill (pos, pos + n, x);
  }

  template <typename InputIterator>
  void
  insert (iterator pos, InputIterator f,
          typename detail::enable_non_integral<InputIterator>::type l)
  {
    assert (pos.vec == this);
    difference_type const count = std::distance (f, l);
    assert (count >= 0);
    just_resize (elems + count);
    std::copy_backward (pos, end () - count, end ());
    std::copy (f, l, pos);
  }

  iterator
  erase (iterator pos)
  {
    assert (pos.vec == this);
    std::copy (pos + 1, end (), pos);
    just_resize (elems - 1);
    return pos;
  }

  iterator
  erase (iterator f, iterator l)
  {
    assert (f.vec == this);
    assert (l.vec == this);
    difference_type count = std::distance (f, l);
    assert (count >= 0);
    std::copy (l, end (), f);
    just_resize (elems - count);
    return f;
  }

  void
  assign (size_type n, value_type t)
  {
    erase (begin (), end ());
    if (n < static_cast<size_type>(elems_per_storage_unit))
      insert (begin (), n, t);
    else
      {
        just_resize (n);

        // Prepare a bit pattern that will be stored directly into storage.
        t &= bit_mask;
        storage_unit_type pattern = 0;
        for (unsigned i = 0; i < static_cast<unsigned>(elems_per_storage_unit);
             ++i)
          {
            pattern <<= bits;
            pattern |= static_cast<storage_unit_type>(t);
          }
        storage_unit_type fixed_pattern = pattern;

        size_type const incomplete_unit_bits
          = storage_unit_bits
          - complete_elems_per_storage_unit * bits;

        size_type lower_incomplete_bits = 0;

        for (typename storage_type::iterator storage_it = storage.begin ();
             storage_it != storage.end (); ++storage_it)
          {
            pattern
              // Lower bits overflowing from previous storage unit.
              = (static_cast<storage_unit_type>(t)
                 >> (bits - lower_incomplete_bits))
              // Middle fixed pattern part with incomplete element.
              | fixed_pattern << lower_incomplete_bits;
            *storage_it = pattern;
            lower_incomplete_bits
              = (bits - (incomplete_unit_bits - lower_incomplete_bits)) % bits;
          }
      }
  }

  template <typename InputIterator>
  void
  assign (InputIterator f,
          typename detail::enable_non_integral<InputIterator>::type l)
  {
    erase (begin (), end ());
    insert (begin (), f, l);
  }

  iterator
  begin ()
  {
    return iterator (this, 0);
  }

  const_iterator
  begin () const
  {
    return const_iterator (this, 0);
  }

  iterator
  end ()
  {
    return iterator (this, elems);
  }

  const_iterator
  end () const
  {
    return const_iterator (this, elems);
  }

  reverse_iterator
  rbegin ()
  {
    return reverse_iterator (end ());
  }

  const_reverse_iterator
  rbegin () const
  {
    return const_reverse_iterator (end ());
  }

  reverse_iterator
  rend ()
  {
    return reverse_iterator (begin ());
  }

  const_reverse_iterator
  rend () const
  {
    return const_reverse_iterator (begin ());
  }

protected:
  friend class detail::packed_vector_value_ref<
    PACKED_VECTOR_TMPL_BR>;

  friend class detail::packed_vector_value_const_ref<
    PACKED_VECTOR_TMPL_BR>;

  typedef typename storage_type::reference storage_unit_ref_type;
  typedef typename storage_type::const_reference storage_unit_const_ref_type;

  size_type
  needed_storage_size () const
  {
    return elems * bits / storage_unit_bits
      + !! (elems * bits % storage_unit_bits);
  }

  static
  size_type
  needed_storage_size (size_type n)
  {
    return n * bits / storage_unit_bits
      + !! (n * bits % storage_unit_bits);
  }

  static
  unsigned
  first_storage_unit (size_type i)
  {
    return static_cast<unsigned>(i * bits / storage_unit_bits);
  }

  static
  unsigned
  shift_in_unit (size_type i)
  {
    return static_cast<unsigned>(i * bits % storage_unit_bits);
  }

  void
  set_elem (size_type i, value_type v)
  {
    // Trim.
    value_type val = v & bit_mask;

    unsigned const shift_1 = shift_in_unit (i);
    unsigned const unit_index = first_storage_unit (i);
    assert (unit_index < storage.size ());
    storage_unit_ref_type unit_1 = storage[unit_index];

    // Lower part or whole element if it fits a single unit.

    // Clear.
    unit_1 &= ~(static_cast<storage_unit_type>(bit_mask) << shift_1);
    // Set.
    unit_1 |= static_cast<storage_unit_type>(val) << shift_1;

    // Higher part.

    int const overflow_len = shift_1 + bits - storage_unit_bits;
    if (overflow_len > 0)
      {
        unsigned const lower_len = bits - overflow_len;
        assert (unit_index + 1 < storage.size ());
        storage_unit_ref_type unit_2 = storage[unit_index + 1];
        // Clear.
        unit_2 &= ~(bit_mask >> lower_len);
        // Set.
        unit_2 |= static_cast<storage_unit_type>(val) >> lower_len;
      }

    assert (v == get_elem (i));
  }

  value_type
  get_elem (size_type i) const
  {
    unsigned const shift_1 = shift_in_unit (i);
    unsigned const unit_index = first_storage_unit (i);
    storage_unit_const_ref_type unit_1 = storage[unit_index];

    // Lower part or whole element if it fits a single unit.

    storage_unit_type val = unit_1 >> shift_1 & bit_mask;

    // Higher part.

    int const overflow_len = shift_1 + bits - storage_unit_bits;
    if (overflow_len > 0)
      {
        unsigned const lower_len = bits - overflow_len;
        storage_unit_const_ref_type unit_2 = storage[unit_index + 1];
        val |= unit_2 << lower_len & bit_mask;
      }

    value_type result;

    if (boost::is_signed<value_type>::value && val > bit_mask / 2)
      result = static_cast<value_type>(val - bit_mask - 1);
    else
      result = static_cast<value_type>(val);

    return result;
  }

  void
  just_resize (size_type n)
  {
    storage.resize (needed_storage_size (n));
    elems = n;
  }

  size_t elems;
  storage_type storage;
};


namespace detail
{

//
PACKED_VECTOR_TMPL_HEADER
inline
packed_vector_value_ref<PACKED_VECTOR_TMPL_BR> const &
packed_vector_value_ref<PACKED_VECTOR_TMPL_BR>::operator =
  (typename packed_vector_value_ref<PACKED_VECTOR_TMPL_BR>::value_type val)
  const
{
  vec->set_elem (index, val);
  return *this;
}


PACKED_VECTOR_TMPL_HEADER
inline
packed_vector_value_ref<PACKED_VECTOR_TMPL_BR> &
packed_vector_value_ref<PACKED_VECTOR_TMPL_BR>::operator =
(packed_vector_value_ref<PACKED_VECTOR_TMPL_BR> const & right)
{
  vec->set_elem (index, right.vec->get_elem (right.index));
  return *this;
}


PACKED_VECTOR_TMPL_HEADER
inline
packed_vector_value_ref<PACKED_VECTOR_TMPL_BR> const &
packed_vector_value_ref<PACKED_VECTOR_TMPL_BR>::operator =
(packed_vector_value_ref<PACKED_VECTOR_TMPL_BR> const & right) const
{
  vec->set_elem (index, right.vec->get_elem (right.index));
  return *this;
}


PACKED_VECTOR_TMPL_HEADER
inline
packed_vector_value_ref<PACKED_VECTOR_TMPL_BR>::operator value_type () const
{
  return vec->get_elem (index);
}


//
PACKED_VECTOR_TMPL_HEADER
inline
packed_vector_value_const_ref<PACKED_VECTOR_TMPL_BR>::operator value_type ()
  const
{
  return vec->get_elem (index);
}

} // namespace detail


PACKED_VECTOR_TMPL_HEADER
inline
bool
operator < (packed_vector<PACKED_VECTOR_TMPL_BR> const & a,
            packed_vector<PACKED_VECTOR_TMPL_BR> const & b)
{
  return std::lexicographical_compare (a.begin (), a.end (), b.begin (),
                                       b.end ());
}


PACKED_VECTOR_TMPL_HEADER
inline
bool
operator == (packed_vector<PACKED_VECTOR_TMPL_BR> const & a,
             packed_vector<PACKED_VECTOR_TMPL_BR> const & b)
{
  if (a.size () == b.size ())
    return std::equal (a.begin (), a.end (), b.begin ());
  else
    return false;
}


#undef PACKED_VECTOR_TMPL_HEADER
#undef PACKED_VECTOR_TMPL_BR


#endif // PACKED_VECTOR_HXX
