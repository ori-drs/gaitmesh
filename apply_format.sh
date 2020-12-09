#!/usr/bin/env sh
find -name '*.cpp' -o -name '*.hpp' | xargs clang-format-3.9 -style=file -i
# skip -o -name '*.h' to avoid PCL header being formatted

