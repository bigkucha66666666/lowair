package com.example.lowairecompany.dao;

import com.example.lowairecompany.pojo.User;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

@Repository  //配置成spring的bean
public interface UserDao extends CrudRepository <User,Integer>{
    User findByUserName(String userName);
    User findByUserNameAndPassword(String userName, String password);
}
