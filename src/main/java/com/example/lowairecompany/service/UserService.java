package com.example.lowairecompany.service;

import com.example.lowairecompany.dao.UserDao;
import com.example.lowairecompany.pojo.User;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service  //标记成sping的bean
public class UserService  implements IUserService{
    @Autowired
    UserDao userDao;
    @Override
    public void add(User user){
        userDao.save(user);
    }
    @Override
    public User getUser(Integer userId) {
        return userDao.findById(userId).orElseThrow(() -> new IllegalArgumentException("用户不存在"));
    }
    @Override
    public User edit(User user){
        return userDao.save(user);
    }
    @Override
    public User delete(Integer userId){
        User existing = userDao.findById(userId).orElseThrow(() -> new IllegalArgumentException("用户不存在"));
        userDao.deleteById(userId);
        return existing;
    }

    @Override
    public User register(User user) {
        if (user.getUserName() == null || user.getPassword() == null) {
            throw new IllegalArgumentException("用户名或密码不能为空");
        }
        User exists = userDao.findByUserName(user.getUserName());
        if (exists != null) {
            throw new IllegalArgumentException("用户名已存在");
        }
        return userDao.save(user);
    }

    @Override
    public User login(String userName, String password) {
        if (userName == null || password == null) {
            throw new IllegalArgumentException("用户名或密码不能为空");
        }
        User user = userDao.findByUserNameAndPassword(userName, password);
        if (user == null) {
            throw new IllegalArgumentException("用户名或密码错误");
        }
        return user;
    }
}
