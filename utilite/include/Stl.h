#ifndef _STL_
#define _STL_

#include <list>
#include <map>
#include <set>
#include <vector>
#include <string>
#include <algorithm>
#include <stdlib.h>

/** \brief Get unique keys from a std::multimap.
  * \param[in] mm The multimap.  
  * \return The list which contains unique keys.
  * \author eddy
  */
template<typename K, typename V>
inline std::list<K> uUniqueKeys(const std::multimap<K, V> & mm) {
	std::list<K> l;
	typename std::list<K>::reverse_iterator lastValue;
	for (typename std::multimap<K, V>::const_iterator iter = mm.begin(); iter!=mm.end(); ++iter) {
		if (iter == mm.begin() || (iter != mm.begin() && *lastValue != iter->first)) {
			l.push_back(iter->first);
			lastValue = l.rbegin();
		}
	}
	return l;
}

/** \brief Get all keys from a std::multimap.
  * \param[in] mm The multimap.  
  * \return The vector which contains all keys (may contains duplicated keys).
  * \author eddy
  */
template<typename K, typename V>
inline std::vector<K> uKeys(const std::multimap<K, V> & mm) {
	std::vector<K> v(mm.size());
	int i=0;
	for (typename std::multimap<K, V>::const_iterator iter = mm.begin(); iter!=mm.end(); ++iter) {
		v[i++] = iter->first;
	}
	return v;
}

/** \brief Get all keys from a std::multimap.
  * \param[in] mm The multimap.  
  * \return The list which contains all keys (may contains duplicated keys).
  * \author eddy
  */
template<typename K, typename V>
inline std::list<K> uKeysList(const std::multimap<K, V> & mm) {
	std::list<K> l;
	for (typename std::multimap<K, V>::const_iterator iter = mm.begin(); iter!=mm.end(); ++iter) {
		l.push_back(iter->first);
	}
	return l;
}

/** \brief Get all values from a std::multimap.
  * \param[in] mm The multimap.  
  * \return The vector which contains all values (contains values from duplicated keys).
  * \author eddy
  */
template<typename K, typename V>
inline std::vector<V> uValues(const std::multimap<K, V> & mm) {
	std::vector<V> v(mm.size());
	int i=0;
	for (typename std::multimap<K, V>::const_iterator iter = mm.begin(); iter!=mm.end(); ++iter) {
		v[i++] = iter->second;
	}
	return v;
}

/** \brief Get all values from a std::multimap.
  * \param[in] mm The multimap.  
  * \return The list which contains all values (contains values from duplicated keys).
  * \author eddy
  */
template<typename K, typename V>
inline std::list<V> uValuesList(const std::multimap<K, V> & mm) {
	std::list<V> l;
	for (typename std::multimap<K, V>::const_iterator iter = mm.begin(); iter!=mm.end(); ++iter) {
		l.push_back(iter->second);
	}
	return l;
}

/** \brief Get values for a specified key from a std::multimap.
  * \param[in] mm The multimap.
  * \param[in] mm The key.
  * \return The list which contains the values of the key.
  * \author eddy
  */
template<typename K, typename V>
inline std::list<V> uValues(const std::multimap<K, V> & mm, const K & key) {
	std::list<V> l;
	std::pair<typename std::multimap<K, V>::const_iterator, typename std::multimap<K, V>::const_iterator> range;
	range = mm.equal_range(key);
	for (typename std::multimap<K, V>::const_iterator iter = range.first; iter!=range.second; ++iter) {
		l.push_back(iter->second);
	}
	return l;
}

/** \brief Get all keys from a std::map.
  * \param[in] m The map.
  * \return The vector of keys.
  * \author eddy
  */
template<typename K, typename V>
inline std::vector<K> uKeys(const std::map<K, V> & m) {
	std::vector<K> v(m.size());
	int i=0;
	for (typename std::map<K, V>::const_iterator iter = m.begin(); iter!=m.end(); ++iter) {
		v[i] = iter->first;
		++i;
	}
	return v;
}

/** \brief Get all keys from a std::map.
  * \param[in] m The map.
  * \return The list of keys.
  * \author eddy
  */
template<typename K, typename V>
inline std::list<K> uKeysList(const std::map<K, V> & m) {
	std::list<K> l;
	for(typename std::map<K, V>::const_iterator iter = m.begin(); iter!=m.end(); ++iter) {
		l.push_back(iter->first);
	}
	return l;
}

/** \brief Get all keys from a std::map.
  * \param[in] m The map.
  * \return The set of keys.
  * \author eddy
  */
template<typename K, typename V>
inline std::set<K> uKeysSet(const std::map<K, V> & m) {
	std::set<K> s;
	int i=0;
	for (typename std::map<K, V>::const_iterator iter = m.begin(); iter!=m.end(); ++iter) {
		s.insert(s.end(), iter->first);
		++i;
	}
	return s;
}

/** \brief Get all keys from a std::map.
  * \param[in] m The map.
  * \return The vector of keys.
  * \author eddy
  */
template<typename K, typename V>
inline std::vector<V> uValues(const std::map<K, V> & m) {
	std::vector<V> v(m.size());
	int i=0;
	for (typename std::map<K, V>::const_iterator iter = m.begin(); iter!=m.end(); ++iter) {
		v[i] = iter->second;
		++i;
	}
	return v;
}

/** \brief Get all keys from a std::map.
  * \param[in] m The map.
  * \return The list of keys.
  * \author eddy
  */
template<class K, class V>
inline std::list<V> uValuesList(const std::map<K, V> & m) {
	std::list<V> l;
	for(typename std::map<K, V>::const_iterator iter = m.begin(); iter!=m.end(); ++iter) {
		l.push_back(iter->second);
	}
	return l;
}

/** \brief Check if the list contains the specified value.
  * \param[in] list The list.
  * \param[in] value The value.
  * \return True if the value is found in the list, otherwise false.
  * \author eddy
  */
template<typename V>
inline bool uContains(const std::list<V> & list, const V & value) {
	return std::find(list.begin(), list.end(), value) != list.end();
}

/** \brief Check if the map contains the specified value.
  * \param[in] map The map.
  * \param[in] key The key.
  * \return True if the key is found in the map, otherwise false.
  * \author eddy
  */
template<typename K, typename V>
inline bool uContains(const std::map<K, V> & map, const K & key) {
	return map.find(key) != map.end();
}

/** \brief Check if the multimap contains the specified value.
  * \param[in] map The multimap.
  * \param[in] key The key.
  * \return True if the key is found in the map, otherwise false.
  * \author eddy
  */
template<typename K, typename V>
inline bool uContains(const std::multimap<K, V> & map, const K & key) {
	return map.find(key) != map.end();
}

/** \brief Check if a string contains a specified substring.
  * \param[in] The string. 
  * \param[in] The specified substring.  
  * \return Ture: has the substring. False: hasn't the substring.
  * \author eddy
  */
inline bool uStrContains(const std::string & _string, const std::string & _substring) {
    return _string.find(_substring) != std::string::npos;
}

#endif  // _STL_