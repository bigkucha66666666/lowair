package com.example.lowairecompany.service;

import com.example.lowairecompany.pojo.User;
import org.springframework.stereotype.Service;


public interface IUserService {
    //增加用户
    void add(User user);
    //查询用户
    User getUser(Integer userId);

    User edit(User user);

    User delete(Integer userId);
    //修改用户
    User register(User user);
    User login(String userName, String password);
}
