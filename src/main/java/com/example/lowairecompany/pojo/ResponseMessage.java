package com.example.lowairecompany.pojo;

import org.springframework.http.HttpStatus;

public class ResponseMessage <T>{
    private Integer code;
    private String message;
    private T data;
    public static <T> ResponseMessage <T> success(T data){
        ResponseMessage responseMessage=new ResponseMessage();
        responseMessage.setCode(HttpStatus.OK.value());
        responseMessage.setMessage("成功");
        responseMessage.setData(data);
        return responseMessage;
    }
    public Integer getCode() {
        return code;
    }

    public void setCode(Integer code) {
        this.code = code;
    }
    public String getMessage() {
        return message;
    }

    public void setMessage(String message) {
        this.message = message;
    }
    public T getData() {
        return data;
    }
    public void setData(T data) {
        this.data = data;
    }
}
