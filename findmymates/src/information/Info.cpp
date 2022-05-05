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

#include "information/Info.h"

namespace information
{
    Info::Info()
    {
    }

    void 
    Info::set_pos(int pos)
    {
        if(pos == 0)
        {
            pos_ = "one";
        }
        else if(pos == 1)
        {
            pos_ = "two";
        }
        else if(pos == 2)
        {
            pos_ = "three";
        }
        else if(pos == 3)
        {
            pos_ = "four";
        }
        else if(pos == 4)
        {
            pos_ = "five";
        }
        else
        {
            pos_ = "six";
        }
    }

    void 
    Info::set_carac(std::string carac, int type)
    {
        if(type == 1)
        {
            name_ = carac;
        }
        else if(type == 2)
        {
            colour_ = carac;
        }
        else 
        {
            object_ = carac;
        }
    }

    std::string Info::get_carac(int type)
    {
        if(type == 1)
        {
            return name_;
        }
        else if(type == 2)
        {
            return colour_;
        }
        else if(type == 3)
        {
            return object_;
        }
        else
        {
            return pos_;
        }
    }

} // namespace information