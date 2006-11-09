################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
$(ROOT)/nbvmain.cpp 

DEPS += \
${addprefix ./, \
nbvmain.d \
}

OBJS += \
./nbvmain.o 


# Each subdirectory must supply rules for building sources it contributes
%.o: $(ROOT)/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	@echo g++ -I/home/gpaul/Desktop/newmat -O0 -g3 -Wall -c -fmessage-length=0 -o$@ $<
	@g++ -I/home/gpaul/Desktop/newmat -O0 -g3 -Wall -c -fmessage-length=0 -o$@ $< && \
	echo -n $(@:%.o=%.d) $(dir $@) > $(@:%.o=%.d) && \
	g++ -MM -MG -P -w -I/home/gpaul/Desktop/newmat -O0 -g3 -Wall -c -fmessage-length=0  $< >> $(@:%.o=%.d)
	@echo 'Finished building: $<'
	@echo ' '


