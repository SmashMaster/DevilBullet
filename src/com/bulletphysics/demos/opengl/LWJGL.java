/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.demos.opengl;

import com.samrj.devil.game.Game;
import com.samrj.devil.game.GameConfig;
import java.awt.event.KeyEvent;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.system.Configuration;

/**
 *
 * @author jezek2
 */
public class LWJGL extends Game {
	
	private static boolean redisplay = false;
	private static LwjglGL gl = new LwjglGL();
	
	public static void postRedisplay() {
		redisplay = true;
	}

	public static IGL getGL() {
		return gl;
	}
	
	public static int main(String[] args, int width, int height, String title, DemoApplication demoApp) throws Exception {
            
            Configuration.DEBUG.set(true);
            
            run(() -> {
                GameConfig config = new GameConfig();
                config.resolution.set(width, height);
                return new LWJGL(title, config, demoApp); 
            });
            return 0;
	}
	
    private final DemoApplication demoApp;
    
    private int mouseX, mouseY;
        
    private LWJGL(String title, GameConfig config, DemoApplication demoApp) throws Exception
    {
        super(title, config);
        
        gl.init();
        
        demoApp.myinit();
        demoApp.reshape(config.resolution.x, config.resolution.y);
        
        this.demoApp = demoApp;
    }
    
    @Override
    public void onKey(int key, int action, int mods)
    {
        int modifiers = 0;
        
        if ((GLFW.GLFW_MOD_SHIFT | mods) != 0) modifiers |= KeyEvent.SHIFT_DOWN_MASK;
        if ((GLFW.GLFW_MOD_CONTROL | mods) != 0) modifiers |= KeyEvent.CTRL_DOWN_MASK;
        if ((GLFW.GLFW_MOD_ALT | mods) != 0) modifiers |= KeyEvent.ALT_DOWN_MASK;
        
        if (action == GLFW.GLFW_PRESS) {
                demoApp.specialKeyboard(key, mouseX, mouseY, modifiers);
        }
        else {
                demoApp.specialKeyboardUp(key, mouseX, mouseY, modifiers);
        }

        if (key == GLFW.GLFW_KEY_ESCAPE) stop();
        if (key == GLFW.GLFW_KEY_Q) stop();
    }
    
    @Override
    public void onCharacter(char character)
    {
        if (character != '\0') {
                demoApp.keyboardCallback(character, mouseX, mouseY, 0);
        }
    }
    
    @Override
    public void onMouseButton(int button, int action, int mods)
    {
        int btn = -1;
        switch (button)
        {
            case GLFW.GLFW_MOUSE_BUTTON_LEFT: btn = 0; break;
            case GLFW.GLFW_MOUSE_BUTTON_MIDDLE: btn = 1; break;
            case GLFW.GLFW_MOUSE_BUTTON_RIGHT: btn = 2; break;
        }
        
        int state = action == GLFW.GLFW_PRESS ? 0 : 1;
        
        
        demoApp.mouseFunc(btn, state, mouseX, mouseY);
    }

    @Override
    public void onMouseMoved(float x, float y, float dx, float dy)
    {
        mouseX = Math.round(x);
        mouseY = getResolution().y - Math.round(y);
        demoApp.mouseMotionFunc(mouseX, mouseY);
    }
    
    @Override
    public void onDestroy()
    {
    }
    
    @Override
    public void render()
    {
        demoApp.moveAndDisplay();
    }
}
