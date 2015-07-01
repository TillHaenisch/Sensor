#
# nbs.ex
#
# Reads sensor data from an XBee module connected to the serial port.
#
# Data ist transmitted in frames with the following format:
#
# <sensor_id> DATA <Vdd> <block number> <humidity> <temperature> <stddev of humidity> <stddev of temp> END
#
# Each packet from a sensor is one line. A line begins with the sensor ID, then the keyword DATA
# then the block number, the power suplly voltage, then the mean temperature,
# the mean humidity, the standard deviation of the temperature and the standard deviation of the humidity
#
# The data values are the raw values from the sensor multiplied by 100 (we don't want to transmit
# floating point numbers)
#
# Till Haenisch till.haenisch@mac.com (c) 2013,2014,2015 All rights reserved
#

defmodule Nbs do
  
    def format_date_time({{year, month, day}, {hour, minute, second}}) do
     :io_lib.format("~4..0B-~2..0B-~2..0B ~2..0B:~2..0B:~2..0B",
       [year, month, day, hour, minute, second])
       |> List.flatten
       |> to_string
    end
  
	def process_file(input_file,current_values) do
	  task = Task.async(fn -> IO.read(input_file, :line) end)
	  try do
	    row = Task.await(task,5000)
		  if (row != :eof) do
		    new_values = process_line(String.split(String.rstrip(row), " "),current_values)
			process_file(input_file,new_values)
	      end
	  catch
	    :exit, _ -> IO.puts "timeout"
		  process_file(input_file,current_values)
	  end
	end
  
    def log(data_point) do
      IO.puts :stderr, format_log(data_point)
    end
		
		def format_display(data_point) do
      {sensor_number, _ , humidity,temp, _ , _} = data_point
    	"#{sensor_number} #{Float.round(humidity,1)}/#{Float.round(temp,1)}"
		end
		
		def format_log(data_point) do
      {sensor_number, date_time, _,_, _ , _} = data_point			
			"New value from from sensor #{sensor_number}, at #{format_date_time(date_time)} #{format_display(data_point)}"
		end
		
		def age(value) do
	  	{_,t,_,_,_,_} = value
	  	{date,time} = :calendar.time_difference(t,:calendar.local_time)
	  	:calendar.time_to_seconds time
		end
		
		def text_form(value,age) when age > 5, do: "---/---"

		def text_form(value,age) do
      format_display(value)
		end
  
		def show_data(values) do
			Enum.map(values,fn(x) -> IO.puts(text_form(x,age(x))) end)
		end
		
  
    def new_list(nil,value,l) do
    	new = l ++ [value]
  	new
    end
  
    def new_list(hit,value,l) do
      {sensor_number,_,_,_,_,_ } = value
    	List.keyreplace(l,sensor_number,0,value)
    end
  		
    def process_line([""],l), do: l
  
    def process_line([sensor_number,"DATA", block_number, num, v_dd, temp, humidity, what_1, what_2, "END"],cur) do
     data_point = {sensor_number, :calendar.local_time, 1.0/16384*String.to_integer(humidity),1.650/16384.0*String.to_integer(temp)-40,v_dd, block_number}
     new_values = new_list(List.keyfind(cur,sensor_number,0),data_point,cur)
     log(data_point)
     now = :calendar.local_time
		 show_data(new_values)
     new_values
     end
   
    def process_line(_,l) do
     IO.puts :stderr, "Error in data"
     l
     end
	 
	 def main(args) do
	     process_file(:stdio,[])
	 end
	 
  end  
  

