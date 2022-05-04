// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

namespace information
{
    Info::Info()
    {
    }

    void 
    set_pos(int pos)
    {
        pos_ = pos;
    }

    int
    get_pos()
    {
        return pos_;
    }

    void 
    set_carac(std::string carac, int type)
    {
        if(type == 0)
        {
            name_ = carac;
        }
        else if(type == 1)
        {
            color_ = carac;
        }
        else
        {
            object_ = carac;
        }
    }

    std::string get_carac(int type)
    {
        if(type == 0)
        {
            return name_;
        }
        else if(type == 1)
        {
            return color_;
        }
        else
        {
            return object_;
        }
    }

} // namespace information