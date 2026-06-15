# Stylized sky — third-party attribution

The sky in `examples/World.tscn` uses the **Stylized Sky** shader and day-sky
material from GDQuest's *godot-4-stylized-sky* project.

- Source: https://github.com/gdquest-demos/godot-4-stylized-sky
- Author: GDQuest (https://www.gdquest.com)
- Files used here (vendored, unmodified except `stylized_sky.tres`'s shader path):
  - `stylized_sky.gdshader`
  - `stylized_sky.tres` (their `sky/examples/day_sky.tres`, repointed to the local shader)

These are **MIT-licensed** (GDQuest's license covers source code, scenes, shaders
and Godot-generated resources under MIT). The cloud/star textures in this material
are procedural Godot resources (FastNoiseLite / GradientTexture2D / CurveTexture),
which are MIT — **none of GDQuest's CC-BY-NC-SA image assets or 3D models are used**,
so this addition keeps GodotWings cleanly MIT-compatible.

The shader also credits Inigo Quilez (MIT) for its Voronoi noise.

## MIT License

Copyright (c) 2023-present GDQuest

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
