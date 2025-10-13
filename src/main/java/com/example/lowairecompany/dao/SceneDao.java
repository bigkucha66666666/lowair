package com.example.lowairecompany.dao;

import com.example.lowairecompany.pojo.Scene;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

@Repository  //配置成spring的bean
public interface SceneDao extends CrudRepository<Scene, Integer> {

}