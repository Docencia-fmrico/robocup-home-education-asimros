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

#ifndef BEHAVIOUR_TREES_INFO_H
#define BEHAVIOUR_TREES_INFO_H

#include <string>

namespace information
{
    class Info
    {
        public:
            Info();

            void set_pos(int pos);

            void set_carac(std::string carac, int type);
            std::string get_carac(int type);

        private:
            std::string pos_; // type 0
            std::string name_; // type 1
            std::string colour_; // type 2
            std::string object_; // type 3
    };

} // namespace information

#endif  // BEHAVIOUR_TREES_INFO_H