#ifndef TYPE_ALIAS_HPP
#define TYPE_ALIAS_HPP

#include<iterator>
#include<vector>
#include<array>
#include<string>
#include<iostream>

namespace vector_types{

using Vec_f=std::vector<float>;
using Poi_f=std::array<float, 2>;
using Vec_Poi=std::vector<Poi_f>;

};

#endif
