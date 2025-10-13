package com.example.lowairecompany.controller;

import com.example.lowairecompany.pojo.ResponseMessage;
import com.example.lowairecompany.pojo.User;
import com.example.lowairecompany.service.IUserService;

import org.hibernate.annotations.processing.Pattern;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.*;
import jakarta.servlet.http.HttpSession;

@RestController    //接口对象返回对象 转换成json文本
@RequestMapping("/user" )    //前端访问接口

public class UserController {
    @Autowired
    IUserService userService;
    //增加
    @PostMapping

    public ResponseMessage add(@RequestBody User user){
        userService.add(user);
        return ResponseMessage.success(null);
    }
    @GetMapping("/{userId}")

    public ResponseMessage get( @PathVariable Integer userId){
        User userNew=userService.getUser(userId);
        return ResponseMessage.success(userNew);
    }
    //修改
    @PutMapping
    public ResponseMessage edit(@Validated @RequestBody User user){
        User userNew=userService.edit(user);
        return ResponseMessage.success(user);
    }
    @DeleteMapping("/{userId}")
    public ResponseMessage delete(@PathVariable Integer userId) {
        User userNew = userService.delete(userId);
        return ResponseMessage.success(userNew);
    }

    // 注册
    @PostMapping("/register")
    public ResponseMessage<User> register(@Validated @RequestBody User user) {
        return ResponseMessage.success(userService.register(user));
    }

    // 登录
    @PostMapping("/login")
    public ResponseMessage<User> login(@RequestParam String userName, @RequestParam String password, HttpSession session) {
        User u = userService.login(userName, password);
        session.setAttribute("loginUser", u);
        return ResponseMessage.success(u);
    }

    // 注销
    @PostMapping("/logout")
    public ResponseMessage<Void> logout(HttpSession session) {
        session.invalidate();
        return ResponseMessage.success(null);
    }

    // 当前登录用户
    @GetMapping("/me")
    public ResponseMessage<User> me(HttpSession session) {
        Object u = session.getAttribute("loginUser");
        return ResponseMessage.success((User) u);
    }

}
