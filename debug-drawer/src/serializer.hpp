#pragma once

#include <vector>
#include <string>
#include <iostream>

class Serializer
{
public:
	Serializer() = delete;

	template <class T>
	static std::vector<T> decode(std::string data)
	{
		std::vector<T> objects;
		uint32_t nbr_of_objs = data.size() / sizeof(T);
		T * obj_ptr = (T *)data.c_str();

		for (size_t i = 0; i < nbr_of_objs; i++)
		{
			objects.push_back(*obj_ptr);
			obj_ptr++;
		}

		return objects;
	}

	template <class T>
	static std::string encode(std::vector<T> objects)
	{
		std::string obj_str = "";

		for (size_t i = 0; i < objects.size(); i++)
		{
			T obj = objects.at(i);
			char *raw = (char *)&obj;
			std::string raw_str(raw, raw + sizeof(T));
			obj_str = obj_str + raw_str;
		}

		return obj_str;
	}

};
