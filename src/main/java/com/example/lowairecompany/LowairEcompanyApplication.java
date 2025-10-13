package com.example.lowairecompany;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.boot.autoconfigure.domain.EntityScan;
import org.springframework.data.jpa.repository.config.EnableJpaRepositories;

@SpringBootApplication
@EntityScan("com.example.lowairecompany.pojo")
@EnableJpaRepositories("com.example.lowairecompany.dao")
public class LowairEcompanyApplication {

    public static void main(String[] args) {
        SpringApplication.run(LowairEcompanyApplication.class, args);
    }

}
