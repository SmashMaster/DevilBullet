# DevilBullet

This is a pure Java port of the [Bullet physics engine](https://github.com/bulletphysics/bullet3). This port is a modified version of JBullet, which was originally written by Martin Dvorak (jezek2@advel.cz) and may be found here:
* http://jbullet.advel.cz/

This version strips out most dependencies, replacing them with [DevilUtil](https://github.com/SmashMaster/DevilUtil). Importantly, the original JBullet relies on javax.vecmath, licensed under GNU GPL v2, which prohibits proprietary use. This version relies only on libraries under permissive licenses.
