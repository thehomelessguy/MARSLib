package com.marslib.util;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;

/**
 * A utility class that uses Java dynamic proxies to automatically construct no-op instances of
 * hardware IO interfaces. This ensures that when in REPLAY mode, the RobotContainer does not need
 * boilerplate anonymous class instantiations, saving lines of code and ensuring all default methods
 * are properly bypassed.
 */
public final class ReplayIOFactory {

  private ReplayIOFactory() {}

  /**
   * Creates a dynamic proxy for the given interface type that does nothing. This is meant to be
   * used for AdvantageKit log replays.
   *
   * @param ioInterface The Class object representing the IO interface.
   * @param <T> The type of the IO interface.
   * @return A no-op instance of the interface.
   */
  @SuppressWarnings("unchecked")
  public static <T> T createProxy(Class<T> ioInterface) {
    return (T)
        Proxy.newProxyInstance(
            ioInterface.getClassLoader(),
            new Class<?>[] {ioInterface},
            new InvocationHandler() {
              @Override
              public Object invoke(Object proxy, Method method, Object[] args) throws Throwable {
                // If it's a default method on the interface, we could invoke it, but commonly
                // no-ops are fine. But let's just return a default value if needed.
                Class<?> returnType = method.getReturnType();
                if (returnType.isPrimitive()) {
                  if (returnType == boolean.class) return false;
                  if (returnType == byte.class) return (byte) 0;
                  if (returnType == short.class) return (short) 0;
                  if (returnType == int.class) return 0;
                  if (returnType == long.class) return 0L;
                  if (returnType == float.class) return 0.0f;
                  if (returnType == double.class) return 0.0d;
                  if (returnType == char.class) return '\0';
                }
                return null;
              }
            });
  }
}
