package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.github.javaparser.StaticJavaParser;
import com.github.javaparser.ast.CompilationUnit;

import org.firstinspires.ftc.teamcode.util.ConversionUtil;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class ExampleUnitTest {

    @Test
    @DisplayName("Test ConversionUtil angle wrapping")
    public void testAngleWrapping() {
        assertEquals(0.0, ConversionUtil.wrapAngleDeg(0.0), 1e-6);
        assertEquals(-170.0, ConversionUtil.wrapAngleDeg(190.0), 1e-6);
        assertEquals(170.0, ConversionUtil.wrapAngleDeg(-190.0), 1e-6);
    }

    @Test
    @DisplayName("Test JavaParser library integration")
    public void testJavaParser() {
        String code = "class TestCode { void hello() {} }";
        CompilationUnit cu = StaticJavaParser.parse(code);
        assertNotNull(cu);
        assertTrue(cu.getClassByName("TestCode").isPresent());
    }
}
