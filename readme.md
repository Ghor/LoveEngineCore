LoveEngine takes the [LÖVE] framework and builds upon it to provide a number of features that can be used to rapidly develop 2D games with extremely low performance overhead.


Notes
-----

The core engine scripts are written with the behavior of LuaJIT's compiler in mind. LuaJIT's FFI library is also used extensively. For these reasons, LoveEngine must be built with LuaJIT and cannot be built with Lua 5.1.


Dependencies
------------

You can get most of these from the [LÖVE sdk].

- SDL2
- OpenGL
- OpenAL
- LuaJIT
- DevIL with MNG and TIFF
- FreeType
- PhysicsFS
- ModPlug
- mpg123
- Vorbisfile

[LÖVE]: http://love2d.org
[LÖVE sdk]: http://love2d.org/sdk
