package com.example.lowairecompany.config;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.annotation.Configuration;
import org.springframework.lang.NonNull;
import org.springframework.web.servlet.config.annotation.InterceptorRegistry;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;

@Configuration
public class WebConfig implements WebMvcConfigurer {

    private final LoginInterceptor loginInterceptor;

    @Autowired
    public WebConfig(LoginInterceptor loginInterceptor) {
        this.loginInterceptor = loginInterceptor;
    }

    @Override
    public void addInterceptors(@NonNull InterceptorRegistry registry) {
        registry.addInterceptor(loginInterceptor)
                .addPathPatterns("/**")
                .excludePathPatterns(
                        "/user/login",
                        "/user/register",
                        "/user/logout",
                        "/swagger-ui.html",
                        "/swagger-ui/**",
                        "/v3/api-docs/**",
                        "/error",
                        "/index.html",
                        "/login.html",
                        "/register.html",
                        "/",
                        "/favicon.ico",
                        "/**/*.css",
                        "/**/*.js",
                        "/**/*.png",
                        "/**/*.jpg",
                        "/**/*.jpeg",
                        "/**/*.gif",
                        "/**/*.svg",
                        "/**/*.woff",
                        "/**/*.woff2"
                );
    }
}
