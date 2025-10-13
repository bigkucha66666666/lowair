package com.example.lowairecompany.config;

import com.example.lowairecompany.pojo.ResponseMessage;
import com.fasterxml.jackson.databind.ObjectMapper;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import jakarta.servlet.http.HttpSession;
import org.springframework.http.HttpMethod;
import org.springframework.http.HttpStatus;
import org.springframework.stereotype.Component;
import org.springframework.web.servlet.HandlerInterceptor;

import java.io.IOException;

@Component
public class LoginInterceptor implements HandlerInterceptor {

    private static final ObjectMapper OBJECT_MAPPER = new ObjectMapper();

    @Override
    public boolean preHandle(HttpServletRequest request, HttpServletResponse response, Object handler) throws Exception {
        if (HttpMethod.OPTIONS.matches(request.getMethod())) {
            return true;
        }
        HttpSession session = request.getSession(false);
        if (session != null && session.getAttribute("loginUser") != null) {
            return true;
        }
        writeUnauthorizedResponse(response);
        return false;
    }

    private void writeUnauthorizedResponse(HttpServletResponse response) throws IOException {
        response.setStatus(HttpStatus.UNAUTHORIZED.value());
        response.setCharacterEncoding("UTF-8");
        response.setContentType("application/json;charset=UTF-8");

        ResponseMessage<Object> message = new ResponseMessage<>();
        message.setCode(HttpStatus.UNAUTHORIZED.value());
        message.setMessage("未登录或登录已过期");
        message.setData(null);

        response.getWriter().write(OBJECT_MAPPER.writeValueAsString(message));
    }
}
